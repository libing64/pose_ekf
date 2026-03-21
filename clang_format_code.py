import os
import string
import argparse
import subprocess


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True)
    parser.add_argument("--exts", default=".h,.cc,.cpp,.hpp,.cu")
    parser.add_argument("--style", default="file")
    parser.add_argument("-sort-includes", default="false")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    exts = map(string.lower, args.exts.split(","))

    for root, subdirs, files in os.walk(args.path):
        for f in files:
            name, ext = os.path.splitext(f)
            if ext.lower() in exts:
                file_path = os.path.join(root, f)
                proc = subprocess.Popen(["clang-format", "--style",
                                         args.style, file_path],
                                        stdout=subprocess.PIPE)

                text = "".join(proc.stdout)

                with open(file_path, "w") as fd:
                    fd.write(text)


if __name__ == "__main__":
    main()
