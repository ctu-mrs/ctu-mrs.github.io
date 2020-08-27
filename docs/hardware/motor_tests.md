---
layout: default
title: Motor tests
parent: Hardware
---

# Motor tests
All motor tests are done with a Turnigy MultiStar BLheli_32 51A ESC, with the 32.7 BLheli firmware. [ESC config](https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/blheli32_esc_config/T650/T650_M1.ini) is available in our repository. 
Current and voltage measurements are done by the ESC and thrust is measured with a 5 kg load cell, coupled with a HX711 amplifier.
Tests are done with fully charged lithium batteries, as we do not have a powerful enough adjustable power supply.

## Plastic 9545 self-tightening propeller

{: .fw-500 }
| Throttle (%) | Thrust (g) | RPM   | Voltage (V) | Current (A) | Power (w) | Efficiency (g/W) |
| :---:        | :---:      | :---: | :---:       | :---:       | :---:     | :---:            |
| 50           | 401        | 5688  | 16.66       | 3.09        | 51.48     | 7.79             |
| 55           | 464        | 6154  | 16.64       | 3.92        | 65.23     | 7.11             |
| 60           | 529        | 6508  | 16.61       | 4.80        | 79.73     | 6.64             |
| 65           | 596        | 6911  | 16.58       | 5.73        | 95.00     | 6.27             |
| 70           | 656        | 7213  | 16.54       | 6.72        | 111.15    | 5.90             |
| 75           | 713        | 7665  | 16.51       | 7.78        | 128.45    | 5.55             |
| 80           | 787        | 7788  | 16.48       | 9.05        | 149.14    | 5.28             |
| 85           | 855        | 8122  | 16.44       | 10.24       | 168.35    | 5.08             |
| 90           | 911        | 8608  | 16.40       | 11.48       | 188.27    | 4.84             |
| 95           | 971        | 8870  | 16.36       | 12.81       | 209.57    | 4.63             |
| 100          | 1024       | 9068  | 16.32       | 14.26       | 232.72    | 4.40             |
{: .fw-700 }


<table>
    <tr>
        <td>Throttle (%)</td>
        <td>Thrust (g)</td>
        <td>RPM</td>
        <td>Voltage (V)</td>
        <td>Current (A)</td>
        <td>Power (w)</td>
        <td>Efficiency (g/W)</td>
    </tr>
    <tr>
        <td>50</td>
        <td>401</td>
        <td>5688</td>
        <td>16.66</td>
        <td>3.09</td>
        <td> 51.48</td>
        <td>7.79</td>
    </tr>
    <tr>
        <td>55</td>
        <td>464</td>
        <td>6154</td>
        <td>16.64</td>
        <td>3.92</td>
        <td> 65.23</td>
        <td>7.11</td>
    </tr>
    <tr>
        <td>60</td>
        <td>529</td>
        <td>6508</td>
        <td>16.61</td>
        <td>4.80</td>
        <td> 79.73</td>
        <td>6.64</td>
    </tr>
    <tr>
        <td>65</td>
        <td>596</td>
        <td>6911</td>
        <td>16.58</td>
        <td>5.73</td>
        <td> 95.00</td>
        <td>6.27</td>
    </tr>
    <tr>
        <td>70</td>
        <td>656</td>
        <td>7213</td>
        <td>16.54</td>
        <td>6.72</td>
        <td> 111.15</td>
        <td>5.90</td>
    </tr>
    <tr>
        <td>75</td>
        <td>713</td>
        <td>7665</td>
        <td>16.51</td>
        <td>7.78</td>
        <td> 128.45</td>
        <td>5.55</td>
    </tr>
    <tr>
        <td>80</td>
        <td>787</td>
        <td>7788</td>
        <td>16.48</td>
        <td>9.05</td>
        <td> 149.14</td>
        <td>5.28</td>
    </tr>
    <tr>
        <td>85</td>
        <td>855</td>
        <td>8122</td>
        <td>16.44</td>
        <td>10.24</td>
        <td> 168.35</td>
        <td>5.08</td>
    </tr>
    <tr>
        <td>90</td>
        <td>911</td>
        <td>8608</td>
        <td>16.40</td>
        <td>11.48</td>
        <td> 188.27</td>
        <td>4.84</td>
    </tr>
    <tr>
        <td>95</td>
        <td>971</td>
        <td>8870</td>
        <td>16.36</td>
        <td>12.81</td>
        <td> 209.57</td>
        <td>4.63</td>
    </tr>
    <tr>
        <td>100</td>
        <td>1024</td>
        <td>9068</td>
        <td>16.32</td>
        <td>14.26</td>
        <td> 232.72</td>
        <td>4.40</td>
    </tr>
</table>
