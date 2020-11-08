#!/bin/bash
echo "generating py file from ui file"
pyside2-uic src/layout/layout.ui > src/layout/layout.py
echo "done"