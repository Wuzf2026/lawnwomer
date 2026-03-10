#!/bin/bash
cd "$(dirname "$0")"/..
tar -czvf lawnwomer_rk3588_$(date +%Y%m%d).tar.gz lawnwomer_ws \
  --exclude=build --exclude=devel --exclude=.git
echo "打包完成！"