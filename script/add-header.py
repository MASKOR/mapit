#!/usr/bin/python
import sys
import subprocess
from collections import namedtuple

if len(sys.argv) < 2:
  sys.exit("No dir given")

use_cmake = False
use_qml = False
use_vert = False
if len(sys.argv) >= 3:
  if sys.argv[2] == 'cmake':
    use_cmake = True
  if sys.argv[2] == 'qml':
    use_qml = True
  if sys.argv[2] == 'vert':
    use_vert = True

proc_find = ""
if use_cmake:
  proc_find = subprocess.Popen(["find " + sys.argv[1] + " \( -name CMakeLists.txt \)"], stdout=subprocess.PIPE, shell=True)
elif use_qml:
  proc_find = subprocess.Popen(["find " + sys.argv[1] + " \( -name \*.qml \)"], stdout=subprocess.PIPE, shell=True)
elif use_vert:
  proc_find = subprocess.Popen(["find " + sys.argv[1] + " \( -name \*.vert \)"], stdout=subprocess.PIPE, shell=True)
else:
  proc_find = subprocess.Popen(["find " + sys.argv[1] + " \( -name \*.h -o -name \*.cpp -o -name \*.hpp -o -name \*.c -o -name \*.cxx -o -name \*.cc -o -name \*.c \)"], stdout=subprocess.PIPE, shell=True)
(find, err) = proc_find.communicate()

for file_name in find.splitlines():
  proc_header = ""
  if use_cmake:
    proc_header = subprocess.Popen(["python generate-licence-header.py " + file_name + " cmake"], stdout=subprocess.PIPE, shell=True)
  else:
    proc_header = subprocess.Popen(["python generate-licence-header.py " + file_name], stdout=subprocess.PIPE, shell=True)
  (header, err) = proc_header.communicate()

  with open(file_name, 'r+') as f:
    content = f.read()
    f.seek(0, 0)
    f.write(header + '\n' + content)
