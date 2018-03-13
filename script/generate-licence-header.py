#!/usr/bin/python
import sys
import subprocess
from collections import namedtuple

if len(sys.argv) < 2:
  sys.exit("No file to check given")

use_cmake = False
if len(sys.argv) >= 3:
  if sys.argv[2] == 'cmake':
    use_cmake = True

proc = subprocess.Popen(["git log --follow --date=short --pretty='%ad %an <%ae>' -- " + sys.argv[1] + " | sed 's/dbulla.*/Daniel Bulla <d.bulla@fh-aachen.de>/'"], stdout=subprocess.PIPE, shell=True)
(out, err) = proc.communicate()
#print "program output:", out

# reverse and remove month and day
outr = ""
for line in out.splitlines():
  outr = line[0:4] + line[10:] + "\n" + outr

# get authors
Author = namedtuple("Author", "name email yearf yeart")
authors = []
for line in outr.splitlines():
  email_start = line.find("<")
  name  = line[5:email_start-1]
  email = line[email_start:]
  year  = line[0:4]

  new_author = True
#  for a in authors:
  for i in range(len(authors)):
    a = authors[i]
    if a.name == name:
      new_author = False
      authors[i] = a._replace(yeart = year)

  if new_author:
    authors.append(Author(name, email, year, year))

output = ""
if use_cmake:
  output = "################################################################################\n#\n"
else:
  output = "/*******************************************************************************\n *\n"

first = True
for a in authors:
  if first:
    first = False
    if use_cmake:
      output += "# Copyright "
    else:
      output += " * Copyright "
  else:
    if use_cmake:
      output += "#           "
    else:
      output += " *           "
  if a.yearf == a.yeart:
    output += "     " + a.yeart
  else:
    output += a.yearf + "-" + a.yeart
  output += " " + a.name + "\t" + a.email + "\n"
if use_cmake:
  output += """#
################################################################################

#  This file is part of mapit.
#
#  Mapit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Mapit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
#"""
else:
  output += """ *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */"""

print output
