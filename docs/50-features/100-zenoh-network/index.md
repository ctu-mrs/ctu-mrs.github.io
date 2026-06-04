---
title: Zenoh router
pagination_label: Zenoh router
description: Zenoh router
---

# How Zenoh router works

Zenoh is a Data Distribution System deloped for fast and scalable data transfer.
The Zenoh router, used by the utilized Zenoh RMW middle-ware, maps the ROS 2 API onto Zenoh API, enabling network communication between multiple ROS 2 hosts.
For in-detail explanation, please refer to the official [design document](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md).

Upon configuring the "rmw_zenoh" middle-ware by setting the variable:
```
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
```
an instance of the Zenoh router executable "rmw_zenohd" is ran with each ROS 2 node.
By default, the [node router configuration](https://github.com/ros2/rmw_zenoh/blob/jazzy/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5) is set to negiotiate peer-to-peer communication between the nodes running on the current host.
As multi-cast scouting is disabled by default, the individual node routers search for a central router which would faciliate the discovery (as "roscore" process did in ROS 1).
Therefore, an instance of the router executable has to be ran manually in each session as:
```
ros2 run rmw_zenoh_cpp rmw_zenohd
```
which by-default utilizes the [host router configuration](https://github.com/ros2/rmw_zenoh/blob/jazzy/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5).

To configure multi-host ROS 2 networking with the MRS system, create a copy of the [MRS router configuration](https://github.com/ctu-mrs/mrs_uav_deployment/blob/ros2/config/zenoh/uav_router.json5) and set the variable to point to the copy:
```
export ZENOH_ROUTER_CONFIG_URI=/path/to/my/custom_uav_router.json5
```

## Configuring network connections

To make Zenoh routers actually route ROS 2 traffic between each other, we have to either enable multi-cast (and connect to random other UAV routers on the same network, do don't) or manually specify their IP addresses.
The routers exchange the routing tables automatically if in 'router' mode, e.g. if you connect your PC to two UAVs, they will be aware of each others nodes.
That can be disabled by setting your PC router in a 'client' mode.

Specify the IP addresses of the other host as:
```yaml
connect : {
  endpoints: ["tcp/192.168.69.101:7447", 
              "tcp/192.168.69.102:7447"],
}
```

## Configure the data transfer

The access control specifies which data is allowed to travel in or out of the Zenoh router using specific paths.

The ROS 2 topics, services or even message types can be filtered using Zenoh key expressions and are declared as "rules".
An example of a full key expression is ```0/robot1/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18```.
Therefore, we make use of wildcard characters to specify the expressions like topics.
An example rule below specifies topics from all ROS domains with any namespace such as ```0/uav1/uav_manager``` to match with ```/diagnostics``` topic with any message type and any message type:
```yaml
"rules": [
  {
    "id": "allow_list",
    "messages": ["put", "declare_subscriber", "declare_queryable", "delete", "query", "reply", "liveliness_token", "declare_liveliness_subscriber", "liveliness_query"],
    "flows": ["ingress", "egress"],
    "permission": "allow",
    "key_exprs": [
    "**/diagnostics/**", 
    "@ros2_lv/**"
    ]
  }
]
```
Make sure to not touch the ```@ros2_lv/**``` expression and allow it on every interface used. It is nescessary for the ROS graph to properly propagate and function.
Services and action topics work exactly the same, but always require two-way flow enabled, in - "ingress" and out - "egress".

The "subjects" specify paths which the data is being filtered on.
The most straighforward way is to specify the network "interfaces":
```yaml
"subjects": [
  {
    "id" : "wifi_interfaces",
    "interfaces" : [ "wlan0" ]
  }
]
```
However, other options are also available, refer to the [design document](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md) for details.
A noteworthy option not mentioned in the design document is the possibility to specify "zids" in a subject, which will allow you to direct data to specific router ids, e.g.:
```yaml
"subjects": [
  {
    "id" : "zids_subject",
    "zids" : [ "1337" ]
  }
]
```
The target router id has to be manually configured in the "id" variable, as it is random by default.
Naturally, this could lead to some very interesting (see multi-cast) networking behavior, so it is still handy to limit the "zids" subject to a specific interface.
Thankfully, the available fields which are "zids", "interfaces", "cert_common_names", and "usernames" can be mixed-and-matched, as illustrated in the [original zenoh router config](https://github.com/ros2/rmw_zenoh/blob/jazzy/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5).
However, do not rely on the examples in the original config indiscriminately, e.g. the key expressions in examples are JUST PLAIN WRONG and they do not reflect the key expression format specified in the design document.
This is reflected by the complete inability of LLM tools to help with configuring zenoh_rmw in general, as most of the sparse examples and presentations on the topic are wrong and the creators essentially poisoned the training data.
It could help to make your favourite AI tool read the design document first before asking any technical questions.

Finally, the "policies" then serve to map the rules onto the subjects:
```yaml
"policies": [
  {
    "id": "allow_list_wifi",
    "rules": ["allow_list"],
    "subjects": ["wifi_interfaces"]
  },
]
```

With this knowledge, you should be good to go.
Keep in mind that as the Zenoh executable keeps running in the background one way or another unless all ROS 2 nodes have cleanly terminated, it is a good practice to always execute ```ros2 daemon stop``` to "flush" the current Zenoh after each change to the config files.

