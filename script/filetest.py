#coding=utf-8
import sys
import os

txtName = "codingWord.txt"

f=open(txtName, "w+")
for i in range(1,100):
        f.write(i)

        f.write("\n")
f.close()