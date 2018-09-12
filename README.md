## PCL Mean and Covariance

Experiments with mean and covariance computation in PCL.

## Installation

1. **Clone this repository** 

   Clone with submodules and pass the `--recursive-submodules` flag:

       $ git clone https://github.com/taketwo/pcl-mean-and-covariance.git --recursive-submodules

2. **Configure with CMake**
    
       $ cd <path to>/pcl-mean-and-covariance
       $ cmake -C CMakeLists.txt -G "Unix Makefiles"

   Note: CMake 3.5 or higher is required. Also note that the generator specified after the `-G` can be something other than Unix Makefiles, E.g. Ninja. 

3. **Install `datamash`** 

   This is required to build and run the benchmark. On later versions of Ubuntu, this can be installed as a prebuilt package with the following command:

       $ sudo apt install datamash

   However on older systems it may be necessary to install `datamash` from source. See the [datamash download page](https://www.gnu.org/software/datamash/download/) for details.

4. **Build and run benchmarks** 

       $ cd ~/bench
       $ make benchmarks

5. **Build and run tests** 

       $ cd ~/tests
       $ make tests

## Troubleshooting

If, when running `make benchmarks` you get the following error:

    Console.cpp:27:20: fatal error: curses.h: No such file or directory
      #include <curses.h>

This is because the `ncurses` module is not installed on your system. You can install it with the command: 
    
    $ sudo apt-get install libncurses5-dev
