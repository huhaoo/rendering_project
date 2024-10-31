#### Compile assimp

> cd ext/assimp
> mkdir build && cd build
> cmake .. -G "Visual Studio 17 2022"

Use VS2022 to open `Assimp.sln`.

#### Compile this project

Init:

> mkdir build
> cd build

Build:

> cmake ..
> cmake --build .

Run:

> Debug\main.exe