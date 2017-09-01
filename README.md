# kinetic_sample_packages
A collection of sample ROS Kinetic nodes for both C++ and Python

## Prerequisites
Before you will be able to compile this package, you will need to ensure you have all the software dependancies installed. It is recommended that you extract the specific example you wish to work with, instead of trying to compile everything at once.

It is assumed you have at least installed the `ros-kinetic-desktop` package.

#### spinner\_py / spinner\_cpp / atime\_sync / custom\_msg\_srv
No extra dependencies needed!

#### image\_sub\_py / image\_processing / solve\_pnp\_py
```sh
sudo apt install ros-kinetic-vision-opencv ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-cv-camera
```

#### mavros\_guider
```sh
sudo apt install ros-kinetic-mavros
```

#### database\_connector
```sh
sudo apt install mysql-server python-mysql.connector
```

## Nodes

#### spinner\_py
This is a example of a full python package, containing a spinning node.

In this example, there is use of a publishers, subscribers, and a timer.

#### spinner\_cpp
This is a example of a full C++ package, containing a spinning node.

In this example, there is use of a publishers, subscribers, and a timer.

#### atime\_sync
An example of using an approximate time policy to subscribe to multiple topics.

This package includes C++ and Python examples.

#### custom\_msg\_srv
Coming soon!

#### image\_sub\_py
A basic example in Python of subscribing to an image, drawing a circle using OpenCV, then publishing the new image.

#### image\_processing
An example in C++ of subscribing to an image, detecting circles with OpenCV, then publishing a new image.

#### solve\_pnp\_py
An example in Python of using the SolvePnP functionality in OpenCV to estimate the distance of circles from the  camera

#### mavros\_guider
An example on how to take control of a UAV using MAVROS, perform a takeoff, then land after a few seconds.

This package includes C++ and Python examples.

#### database\_connector
An example on how to create and use a database connection.

This package includes C++ and Python examples.

It is assumed that you have the follow:
- A mysql user called "user" with the password "mysql"
- A mysql database called "ros\_db"
- A table in _ros\_db_ called "gas_data", with two columns: "timestamp" and "reading"

## Repurposing Examples
If you would like to use the example packages as a base for your own Nodes, you will have to rename them at the very least.

Here is a list of files needed to rename the _spinner\_py_ example. We will rename it to "my_package".

- Package Folder
  - Rename to "my_package"
- README.md
  - Change title to my_package
  - Add in a brief desctiption
- package.xml
  - `<name>` tag to my_package
  - `description` to match my_package function
- CMakeLists.txt
  - `project()` to match my_package
  - `install()` update to look for your node (e.g. scripts/my_package_node)
- setup.py
  - `packages=[]` to match my_package
- src/kinetic\_sample_spinner\_py
  - Rename folder to "my_package"
- src/kinetic\_sample_spinner\_py/spinner
  - Rename file to "my_class.py"
  - `class Spinner()` to `class MyClass()`
- scripts/spinner\_node
  - Rename file to "my_node"
  - `from kinetic_sample_spinner_py.spinner import Spinner` update to resemble your src folder (e.g. `from my_package.my_class import MyClass`)
- launch/spinner.launch
  - Rename and customize

Here is a list of files needed to rename the _spinner\_cpp_ example. We will rename it to "my_package".

- Package Folder
  - Rename to "my_package"
- README.md
  - Change title to my_package
  - Add in a brief desctiption
- package.xml
  - `<name>` tag to my_package
  - `description` to match my_package function
- CMakeLists.txt
  - `project()` to match my_package
  - `install()` update to look for your node (e.g. scripts/my_package_node)
  - `add_library(spinner src/spinner/spinner.cpp)` to add your class (e.g. `add_library(myClass src/myClass/myClass.cpp)`)
  - `add_dependencies(spinner ...` to match `add_dependencies(myClass`
  - `add_executable(spinner_node src/spinner_node.cpp)` declare a node `add_executable(my_node src/my_node.cpp)`
  - `add_dependencies(spinner_node ...` to match `add_dependencies(my_node ...`
  - `target_link_libraries(spinner_node spinner ...` to match `target_link_libraries(my_node myClass ...`
- include/spinner
  - Rename to "myClass"
- include/spinner/spinner.h
  - Rename to "myClass.h"
  - `class Spinner` to match `class MyClass`
  - Update constructors and destructors to match
- src/spinner
  - Rename to "myClass"
- src/spinner/spinner.cpp
  - `#include "spinner/spinner.h"` to `#include "myClass/myClass.h"`
  - `Spinner::` to match `MyClass::`
  - Update constructors and destructors to match
- src/spinner_node.cpp
  - `#include "spinner/spinner.h"` to `#include "myClass/myClass.h"`
  - `Spinner sp;` to match `MyClass mc;`
- launch/spinner.launch
  - Rename and customize
