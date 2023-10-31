---
layout: default
title: Nimbro Explained
parent: Software
---

# How Nimbro Works

The way Nimbro works is that it spawns several sockets that connect the IP addresses of all the robots in a network. It uses TCP and UDP communication to forward topics and services to these. It does this by spawning many nodes/agents on each host and these agents relay information on the ports which are then converted to topics and services. There are some nuances about how topics and services work and how they are different that are relevant.

## Topics

Topics work slightly differently to services. When a topic is published on Host1 as `/Host1/<topic_name>/<topic_subname>`, Nimbro has to be informed on Host1 through configuration file to listen to this topic and republish it on the other side on Host2. However, Host2 doesn't need to contain this configuration because this topic name is sent from one nimbro node to another nimbro node, and it can publish this topic using the topic name contained in the UDP or TCP packet.

This configuration can be done in two ways:
- Topic: `<topic_name>/<topic_subname>`: If specified without a leading slash, nimbro will forward your topic to other hosts and they will show up as `/Host1/<topic_name>/<topic_subname>`.
- Topic: `/<topic_name>/<topic_subname>`: If specified with a leading slash, nimbro will forward the topic as it is and it will show up as `/<topic_name>/<topic_subname>` and your hostname won't be attached to the topic. **Therefore, this is the only way to publish topics to other hosts and you can choose what it looks like by adding a leading slash.**

## Services

Services are slightly different because the information for which service to call and how to get the response is not contained in the UDP/TCP packet and therefore the configuration needs to be done in all the hosts. Services differ in another way that each service requires two separate ports since this is a two-way communication. Also, a single nimbro node serves only one service.


# Nimbro Failures

One of the key way Nimbro can fail is if you don't have ports unblocked when it starts. Nimbro needs some port series to be unblocked and you can check both the port series and their status in the Nimbro launch pane. You can then modify the series according to your needs. They are currently set at 17XXX and 6XXX.
