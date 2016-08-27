# f1_10_adv_alg

mLab experimental algorithms for autonomous racing. 

See INSTALL for full system installation instructions.


## License

## Hardware

- Jetson TK-1

## Requirements

- ROS indigo (Ubuntu 14.04)
- CUDA (Optional)

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-indigo-ros-base ros-indigo-tf libeigen3-dev libmrpt-dev ros-indigo-map-msg ros-indigo-laser-geometry ros-indigo-pcl-conversions ros-indigo-pcl-ros
% sudo apt-get install ros-indigo-hokuyo-node ros-indigo-um7
```

### How To Build

```
cd $HOME
$ git clone http://github.com/mlab-upenn/f1_10_adv_alg.git
$ cd ~/f1_10_adv_alg/src
$ catkin_init_workspace
$ cd ../
$ catkin_make -j1
```

## How to Start

```
sudo apt-get install tmux ruby
sudo gem install tmuxinator
cp ~/f1_10_adv_alg/start_adv_arc.yml ~/.tmuxinator/
tmuxinator start adv_arc
```
