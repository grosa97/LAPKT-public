import argparse
import sys

from os.path import basename, abspath, dirname, join, exists

FILEDIR_ABSPATH = abspath(dirname(__file__))

def get_include_dirpath() -> str:
    """
    Path to the lapkt include directory
    """
    include_dirpath = join(FILEDIR_ABSPATH, "core", "include")

    if exists(include_dirpath):
        return include_dirpath
    else:
        raise ImportError("lapkt is not installed correctly!")

def get_cmake_dirpath() -> str:
    """
    Path to the lapkt cmake module directory
    """
    lapkt_path = join(FILEDIR_ABSPATH, "cmake")
    if exists(lapkt_path):
        return lapkt_path
    else:
        raise ImportError("lapkt is not installed correctly!")

def main() -> None:
    """
    """
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--includedir",
        action="store_true",
        help="include directory for lapkt headers",
    )
    parser.add_argument(
        "--cmakedir",
        action="store_true",
        help="lapkt cmake module directory",
    )

    args = parser.parse_args()
    # Print help if no argument is provided
    if not sys.argv[1:]:
        parser.print_help()
    else:
        # return include dir path
        if args.includedir:
            print(get_include_dirpath())
        # return cmake dir path
        if args.cmakedir:
            print(get_cmake_dirpath())

if __name__ == "__main__":
    main()
