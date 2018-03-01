#!/usr/bin/python
import sys
import subprocess
from collections import namedtuple

if len(sys.argv) < 2:
  sys.exit("No dir given")

proc_find = subprocess.Popen(["find " + sys.argv[1] + " \( -name \*.h -o -name \*.cpp -o -name \*.hpp -o -name \*.c -o -name \*.cxx -o -name \*.cc -o -name \*.c \)"], stdout=subprocess.PIPE, shell=True)
(find, err) = proc_find.communicate()

for file_name in find.splitlines():
#  proc_cat = subprocess.Popen(["cat " + file_name], stdout=subprocess.PIPE, shell=True)
#  (cat, err) = proc_cat.communicate()
#  
  proc_header = subprocess.Popen(["python generate-licence-header.py " + file_name], stdout=subprocess.PIPE, shell=True)
  (header, err) = proc_header.communicate()

  with open(file_name, 'r+') as f:
    content = f.read()
    f.seek(0, 0)
    f.write(header + '\n' + content)
