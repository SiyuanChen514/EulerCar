# !/usr/bin/env python
# -*- coding: utf-8 -*-

# @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
# disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
# following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
# products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ide_entry.py Function implementation: ide build entry file, which is used to 
# copy code and invoke build compilation scripts.

import os
import sys
import stat
import pathlib
import glob
import tarfile
import zipfile
import platform
import shutil
from configparser import ConfigParser
import shlex
import subprocess
from build import remove_readonly

from build_gn import read_json_file


def copy_code(product, copy_path):
    '''
    Function description: Code copy phase for CI build work.
    '''

    # Instantiation parameter check.
    if not isinstance(copy_path, str):
        raise TypeError("copy_path in para type error {}".format(
                        type(copy_path)))
    if not isinstance(product, str):
        raise TypeError("product in para type error {}".format(
                        type(product)))

    if pathlib.Path(copy_path).exists():
        shutil.rmtree(os.path.realpath(copy_path), onerror=remove_readonly)

    copy_abspath = pathlib.Path(copy_path).resolve()
    copyjson_path = pathlib.Path.cwd()\
                    .joinpath('chip', "{}".format(product), 'codecopy.json')
    copyjson_content = read_json_file(copyjson_path)
    for module_path in copyjson_content['modules']:
        if pathlib.Path(module_path).is_dir():
            traversal_path(copy_abspath, module_path)
        elif pathlib.Path(module_path).is_file():
            parent_path = pathlib.Path(module_path).parent
            copy_parent_path = pathlib.Path(copy_abspath).joinpath(parent_path)
            if not copy_parent_path.exists():
                os.makedirs(copy_parent_path)
            shutil.copy(module_path, copy_parent_path)


def traversal_path(copy_path, module_path):
    '''
    Function description: Traverse the path and then perform the create 
    and copy work.
    '''

    ipbefore_list = []
    ipafter_list = []
    cur_sys = platform.system()

    for (dirpath, _, filenames) in os.walk(module_path):
        if cur_sys == 'Windows':
            ipbefore_list = dirpath.split('\\')
        elif cur_sys == 'Linux':
            ipbefore_list = dirpath.split('/')
        ipafter_list = []
        for ipdir in ipbefore_list:
            if ipdir.startswith('v') and '.' in ipdir:
                continue
            ipafter_list.append(ipdir)
        dirprocesspath = '/'.join(ipafter_list)
        dest_path = pathlib.Path(copy_path).joinpath(dirprocesspath)
        if not dest_path.exists():
            os.makedirs(dest_path)
        for file in filenames:
            if 'entry.py' in file or 'trustlist.json' in file:
                continue
            source_file = pathlib.Path(dirpath).joinpath(file)
            shutil.copy(source_file, dest_path)


def untar(filename):
    '''
    Function description: Decompress the tar or tar.gz file.
    '''

    tar = tarfile.open(filename)
    tar.extractall(pathlib.Path())
    tar.close()


def unzip(filename):
    '''
    Function description: Decompress the zip file.
    '''

    max_size = 1 * 1024 * 1024 * 500
    cur_size = 0

    zip_file = zipfile.ZipFile(filename)
    filename = filename.split('.')[0]
    if not os.path.isdir(filename):
        os.mkdir(filename)
    for names in zip_file.infolist():
        cur_size += names.file_size
        if cur_size > max_size:
            break
        zip_file.extract(names.filename, filename)
    zip_file.close()


def del_decfile(tools_path):
    '''
    Function description: Delete the decompressed files.
    '''

    file_lst = glob.glob(tools_path + '/*')
    filename_lst = [os.path.basename(i) for i in file_lst if '.' not in os.path.basename(i)]
    for filename in filename_lst:
        shutil.rmtree(filename, onerror=remove_readonly)


def un_alltools(tools_path):
    '''
    Function description: Decompress all compilation tools.
    '''

    # Judgment system.
    cur_sys = platform.system()
    path = str(pathlib.Path(tools_path).joinpath(cur_sys))
    cur_path = os.getcwd()

    os.chdir(path)
    del_decfile(path)

    # Decompress all compressed files.
    file_lst = glob.glob(path + '/*')
    filename_lst = [os.path.basename(i) for i in file_lst]
    for filename in filename_lst:
        if '.' in filename:
            suffix = filename.split('.')[-1]
            if suffix == 'gz' or suffix == 'tar':
                untar(filename)
            if suffix == 'zip':
                unzip(filename)
    os.chdir(os.path.realpath(cur_path))


def ide_entry(argv):
    '''
    Function description: ci entry function.
    '''

    # Save the path of the full package tool.
    tools_path = str(pathlib.Path().cwd().joinpath('tools', 'toolchain'))
    if len(argv) == 3:
        copy_chip = argv[1]
        copy_path = argv[2]
    else:
        copy_chip = '3065h'
        copy_path = '../mcu_pro'

    # Decompress all compilation tools.
    un_alltools(tools_path)

    # Detach the chip package.
    copy_code(copy_chip, copy_path)

    os.chdir(pathlib.Path(copy_path).resolve())
    # Write the path of the full package tool to the config.ini file.
    config_path = pathlib.Path().cwd().joinpath('build', 'config.ini')
    config = ConfigParser()
    config.read(config_path)
    config.set('gn_args', 'tools_path', tools_path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(config_path, flags, modes), 'w+') as configini:
        config.write(configini)

    os.makedirs("ohos_bundles")


if __name__ == "__main__":
    sys.exit(ide_entry(sys.argv))
