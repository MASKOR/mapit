---
layout: page
permalink: /operator/load_rosbags
---

## operator/load_rosbags

### Requires
- Eigen3

### Parameters (JSON)
- map: the map to store the transforms within
- transforms: an array of objects
   - static: whenever this tf is static or not
   - header: Data with is stored in the entity
      - frame_id
      - stamp: an object
         - sec
         - nsec
   - transform: Data which is stored in the entity data
      - child_frame_id
      - translation
         - x
         - y
         - z
      - rotation: an object
         - w
         - x
         - y
         - z

optional:

- dynamic_layer: the name for dynamic tfs
- static_layer: the name for static tfs

```javascript
{
 "map" : ...,
 "dynamic_layer" : ...,  [optional]
 "static_layer" : ...,   [optional]
 "transforms" :
 [                       [repeated]
     {
         "static" : ...,
         "header" : {
             "frame_id" : ...,
             "stamp" : {
                 "sec" : ...,
                 "nsec" : ...
             }
         },

         "transform" : {
             "child_frame_id" : ...,
             "translation" : {
                 "x" : ...,
                 "y" : ...,
                 "z" : ...
             },
             "rotation" : {
                 "w" : ...,
                 "x" : ...,
                 "y" : ...,
                 "z" : ...
             }
         }
     },
     {
         ...
     }
 ]
}

```

### Effect
Loads a list of transforms into the system
