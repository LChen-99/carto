#!/bin/sh
###
 # @Author: LuoChen 1425523063@qq.com
 # @Date: 2022-12-08 15:32:55
 # @LastEditors: LuoChen 1425523063@qq.com
 # @LastEditTime: 2023-01-04 09:36:35
 # @FilePath: /catkin_ws/src/cartographer/scripts/install_async_grpc.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 

# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

git clone https://github.com/cartographer-project/async_grpc
cd async_grpc
git checkout 771af45374af7f7bfc3b622ed7efbe29a4aba403
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  ..
ninja
sudo ninja install
