---
layout: page
permalink: /tool/stream_to_ros
---

## tool/stream\_to\_ros

streams data from mapit (entities or layers) to ROS topics

### Usage

```
mapit stream_to_ros <workspace name>:
  -h [ --help ]                  print usage
  -w [ --workspace ] arg         the workspace (formerly checkout) to work with
  -s [ --use_sim_time ] arg (=0) whenever the clock should be published or not.
                                 When entities are shown, this param will be 
                                 ignored.
                                 (Only usefull in the "playback mode" which is 
                                 only availible when layesr are displayed)
  -m [ --map ] arg               the map to work with
  -l [ --layers ] arg            When specified, this will be used and the 
                                 entities options will be ignored.this can be 
                                 (if set) one ore more layer.
                                 E.g. "layer_1" "layer_2" ...
                                 
                                 Data will be displayed in a "playback mode" 
                                 (published to there relativ timestamps)
  -e [ --entities ] arg          When layers is used, this will be ignored.this
                                 can be (if set) specific layer/entity pairs.
                                 E.g. "layer_1 entity_1 entity_2 ..."
                                      "layer_2 entity_8 entity_5 ..."
                                       ...
                                 
                                 Data will be displayed "all at once" 
                                 (timestamps will be ignored)
  --repository-directory arg     directory to store data locally
  --url arg                      remote repository url
  --compute-local                only if remote repository with option "--url" 
                                 is used

```
