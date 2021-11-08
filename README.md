# Edith_Visualization

Visualization for Ediths Magic Pipeline

## 1. build and run docker container

add username und password in line 69,71 and 72 to have access like:

`git clone https://username:password@rgit.acin.tuwien.ac.at/christianxeder/edith_visualization.git`

```
cd edith_visualization_service/docker
docker build -t "vis" . 
run ./sh
```


## 2. Start mongodb

`roslaunch --wait mongodb_store mongodb_store.launch db_path:=/path/to/mongodb`

## 3. Start Visualization program
`rosrun edith_visualization_service visualization.py`

## 4. Controll Visualization program via services

Publish Pointcloud: `rosservice call /edith_visualization_service/visualize`

Clear Pointcloud: `rosservice call /edith_visualization_service/clear`

Stop Publish Pointcloud: `rosservice call /edith_visualization_service/stop`


