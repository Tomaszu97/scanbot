#!/bin/bash
echo "generating py file from ui file"
pyside2-uic layout/layout.ui > layout/layout.py
echo "done"