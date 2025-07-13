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
# build.py Function implementation: Build the compilation framework and 
# compile the project.

import sys
import os
import pathlib
import argparse
import collections
import shutil
import subprocess
import distutils.spawn
from configparser import ConfigParser
import platform
import logging
import stat
import shlex

from build_gn import read_json_file, del_allgn, AutoCreate


def usage():
    '''
    Function description: Compiling Commands lists.
    '''

    msg = "\n  python build/build.py\n"\
          "  python build/build.py build\n"\
          "  python build/build.py checkbuild\n"\
          "  python build/build.py -t hcc_fpu\n"\
          "  python build/build.py -b debug\n"
    return msg


def copy_xml():
    '''
    Function description: Copy xml file to out.
    '''
    build_tmp_path = pathlib.Path.cwd().joinpath('build', 'createxml',
                                                 'mss_prim_db')
    xml_path = build_tmp_path.joinpath('dfx_db', 'log.xml')
    target_path = pathlib.Path.cwd().joinpath('out')
    shutil.copy(xml_path, target_path)
    if os.path.isdir(build_tmp_path):
        shutil.rmtree(build_tmp_path)
 
 
def generatefile(file_path, config):
    '''
    Function description: Signing Executable Files.
    '''
    # Instantiation parameter check.
    if not isinstance(file_path, str):
        raise TypeError("file_path in para type error {}".format(
                        type(file_path)))

    file_abspath = pathlib.Path(file_path).resolve()
    # Generate the bin file.
    bin_abspath = file_abspath.parent.joinpath('{}.bin'
                                               .format(file_abspath.stem))
    cmd = ['riscv32-linux-musl-objcopy', '-Obinary', str(file_abspath), str(bin_abspath)]
    process = subprocess.Popen(cmd, shell=False)
    process.wait()
    ret_code = process.returncode
    if ret_code != 0:
        raise Exception("bin_file failed, return code is " + ret_code)

    # Generate the hex file.
    hex_abspath = file_abspath.parent.joinpath('{}.hex'
                                               .format(file_abspath.stem))
    cmd = ['riscv32-linux-musl-objcopy', '-Oihex', str(file_abspath), str(hex_abspath)]
    process = subprocess.Popen(cmd, shell=False)
    process.wait()
    ret_code = process.returncode
    if ret_code != 0:
        raise Exception("hex_file failed, return code is " + ret_code)
    
    # Generate the list file.
    if config.build_type == 'debug':
        list_path = file_abspath.parent.joinpath('{}.list'
                                                 .format(file_abspath.stem))
        if list_path.exists():
            os.remove(list_path)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        modes = stat.S_IWUSR | stat.S_IRUSR
        with os.fdopen(os.open(list_path, flags, modes), 'w+') as list_file:
            cmd = ['riscv32-linux-musl-objdump', '-S', str(file_abspath)]
            process = subprocess.Popen(cmd, stdout=list_file, shell=False)
            process.wait()
            ret_code = process.returncode
            if ret_code != 0:
                raise Exception("list_file failed, return code is " + ret_code)
    else:
        cmd = ['riscv32-linux-musl-strip', str(file_abspath)]
        process = subprocess.Popen(cmd, shell=False)
        process.wait()
        ret_code = process.returncode
        if ret_code != 0:
            raise Exception("strip failed, return code is " + ret_code)


def run_build(**kwargs):
    '''
    Function description: Start building.
    '''

    config = kwargs.get('config')
    compile_var = Compile()
    compile_var.compile(config)
    file_path = str(pathlib.Path().joinpath('out',
                                                'bin', 'target.elf'))
    generatefile(file_path, config)


def exec_command(cmd, log_path, **kwargs):
    '''
    Function description: Run the build command.
    '''
    flags = os.O_WRONLY | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(log_path, flags, modes), 'w') as log_file:
        process = subprocess.Popen(cmd,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   universal_newlines=True,
                                   errors='ignore',
                                   **kwargs)
        # Write the build process to the build.log.
        for line in iter(process.stdout.readline, ''):
            log_file.write(line)

    process.wait()
    ret_code = process.returncode

    # An error code is returned when the command is executed.
    if ret_code != 0:
        with os.fdopen(os.open(log_path, flags, modes), 'at') as log_file:
            for line in iter(process.stderr.readline, ''):
                log_file.write(line)
        raise Exception("{} failed, return code is {}".format(cmd, ret_code))


def parsejson_startautocreat(config):
    '''
    Function description: Parsing Chip Template Files.
    '''

    # Obtaining the gn and ninja Paths.
    Compile.get_tool_path()
    # Read the content of the compilation configuration file.
    product_json = pathlib.Path.cwd().joinpath('chip', 'target',
                                               'userconfig.json')
    json_content = read_json_file(product_json)

    if config.action == 'checkbuild':
        del_output(config)
    check_output(config)
    check_extcomponent()
    del_allgn()
    AutoCreate(json_content)
    return True


def makedirs(path, exist_ok=True):
    '''
    Function description: Creating a directory.
    '''

    try:
        os.makedirs(path)
    except OSError:
        if not pathlib.Path(path).is_dir():
            raise Exception("{} makedirs failed".format(path))
        if not exist_ok:
            raise Exception("{} exists, makedirs failed".format(path))
    finally:
        pass


def remove_readonly(func, path, _):
    '''
    Function description: Change the read-only permission to write.
    '''

    os.chmod(path, stat.S_IWRITE)
    func(path)


def del_output(config):
    '''
    Function description: Delete output path.
    '''

    out_path = config.get_out_path()
    bin_path = pathlib.Path(out_path).joinpath('bin')
    libs_path = pathlib.Path(out_path).joinpath('libs')
    obj_path = pathlib.Path(out_path).joinpath('obj')
    product_json = pathlib.Path.cwd().joinpath('chip', 'target',
                                               'userconfig.json')
    lib_name = ""

    json_content = read_json_file(product_json)
    if json_content['system'][0]['subsystem'][0]['component'][0].get('name'):
        lib_name = json_content['system'][0]['subsystem'][0]['component'][0]\
                   .get('name')
        lib_name = "lib{}.a".format(lib_name)

    for (dirpath, _, filenames) in os.walk(bin_path):
        for file in filenames:
            if "target" in file or "allinone" in file:
                os.remove(pathlib.Path(dirpath).joinpath(file))

    for (dirpath, _, filenames) in os.walk(libs_path):
        for file in filenames:
            if file == lib_name:
                os.remove(pathlib.Path(dirpath).joinpath(file))

    for (dirpath, _, filenames) in os.walk(obj_path):
        for file in filenames:
            if pathlib.Path(file).suffix == '.o':
                os.remove(pathlib.Path(dirpath).joinpath(file))


def check_output(config):
    '''
    Function description: Recreate output path.
    '''
    out_path = config.get_out_path()
    if not pathlib.Path(out_path).exists():
        makedirs(out_path)


def check_extcomponent():
    '''
    Function description: Deletes output files generated by 
    external components.
    '''

    ext_inc_path = pathlib.Path('middleware').joinpath(
                   'thirdparty', 'sysroot', 'include')
    ext_lib_path = pathlib.Path('middleware').joinpath(
                   'thirdparty', 'sysroot', 'lib')
    if not ext_inc_path.exists():
        makedirs(ext_inc_path)
    if not ext_lib_path.exists():
        makedirs(ext_lib_path)


def config_create(**kwargs):
    '''
    Function description: Start to create configuration.
    '''

    config = kwargs.get('config')
    parsejson_startautocreat(config)
    return True


def exec_create(args):
    '''
    Function description: Start creating the compilation process.
    '''

    callback_dict = CallbackDict()

    # parse action
    if args.action[0] == 'build' or\
       args.action[0] == 'checkbuild':
        config = Config(args)
        callback_dict.register(config.action, config_create)
        callback_dict.register(config.action, run_build)
        callback_dict.execute(config.action,
                             config=config,
                             args=args)
    elif args.action[0] == 'clean':
        config = Config(args)
        del_output(config)
        del_allgn()
    else:
        raise Exception("Error: action not found.")


class Config():
    '''
    Function description: config config.ini.
    '''

    def __init__(self, args):
        self.action = args.action[0]
        self.build_type = args.build_type[0]
        self.tool_chain = args.tool_chain[0]
        self.__set_path()
        self.config = pathlib.Path(self.get_build_path())\
                      .joinpath('config.ini')
        self.log_path = pathlib.Path(self.get_out_path()).joinpath('build.log')
        self.cfg = ConfigParser()
        self.cfg.read(self.config)
        self.toolenv_check()
        self.set_default_cmd()
        self.set_env_path()
        self.args_list = []

    def get_root_path(self):
        if self.__root_path is None:
            raise Exception('Error: set root_path first.')

        return self.__root_path

    def get_build_path(self):
        if self.__build_path is None:
            raise Exception('Error: set build_path first.')

        return self.__build_path

    def get_out_path(self):
        if self.__out_path is None:
            raise Exception('Error: set out_path first.')

        return self.__out_path

    def toolenv_check(self):
        '''
        Function description: Check whether the tool chain path is set.
        '''

        toolspath = self.cfg.get('gn_args', 'tools_path')
        user_tool_path = os.path.join(os.path.expanduser("~"), 
            ".deveco-device-tool/compiler_tool_chain")
        if not os.path.exists(user_tool_path):
            # use default tool chain path
            return

        if toolspath != user_tool_path:
            self.cfg.set('gn_args', 'tools_path', user_tool_path)
            flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
            modes = stat.S_IWUSR | stat.S_IRUSR
            with os.fdopen(os.open(self.config, flags, modes),
                            'w') as configini:
                self.cfg.write(configini)
        elif not toolspath:
            raise Exception("Error: please set config.ini tools_path.")

    def set_default_cmd(self):
        '''
        Function description: Write the toolchain and version information to 
        the config.ini file.
        '''

        section = 'gn_args'
        userconfig_file_name = 'userconfig.json'
        default_build_type = self.cfg.get(section, 'build_type')
        default_tool_chain = self.cfg.get(section, 'toolchain_select')
        target_path = pathlib.Path(self.get_root_path()).joinpath('chip',
                                                                  'target')
        compileopt_path = pathlib.Path(self.get_build_path())\
                          .joinpath('config')

        if self.build_type != default_build_type:
            if self.build_type != 'debug' and\
               self.build_type != 'release':
                raise Exception('Error: {} is not build_type, please check.'\
                                .format(self.build_type))
            self.cfg.set(section, 'build_type', self.build_type)
        if self.tool_chain != default_tool_chain:
            if self.tool_chain != 'hcc' and\
               self.tool_chain != 'hcc_fpu':
                raise Exception('Error: {} is not tool_chain, please check.'\
                                .format(self.tool_chain))
            # Updating the userconfig.json File.
            shutil.copy(pathlib.Path(compileopt_path)\
                        .joinpath(self.tool_chain, userconfig_file_name),
                        target_path)
            self.cfg.set(section, 'toolchain_select', self.tool_chain)
        if not pathlib.Path(target_path).joinpath(userconfig_file_name).exists():
            # userconfig.json file corresponding to different tool chains.
            shutil.copy(pathlib.Path(compileopt_path)\
                        .joinpath(self.tool_chain, userconfig_file_name),
                        target_path)
        if self.build_type != default_build_type or\
           self.tool_chain != default_tool_chain:
            flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
            modes = stat.S_IWUSR | stat.S_IRUSR
            with os.fdopen(os.open(self.config, flags, modes),
                           'w') as configini:
                self.cfg.write(configini)

    def set_env_path(self):
        '''
        Function description: Write the toolchain and version information to 
        the config.ini file.
        '''

        compiler_path = None
        tools_path = self.cfg.get('gn_args', 'tools_path')

        cur_sys = platform.system()
        if cur_sys == "Linux":
            gn_name = 'gn-linux'
            ninja_name = 'ninja-linux'
            hcc_name = 'cc_riscv32_musl'
            hccfpu_name = 'cc_riscv32_musl_fp'
        elif cur_sys == "Windows":
            gn_name = 'gn-win'
            ninja_name = 'ninja-win'
            hcc_name = 'cc_riscv32_musl_win'
            hccfpu_name = 'cc_riscv32_musl_fp_win'
        # Setting GN and NINJA env.
        gn_path = pathlib.Path(tools_path).joinpath(cur_sys, gn_name)
        ninja_path = pathlib.Path(tools_path).joinpath(cur_sys, ninja_name)
        # Setting Toolchain env.
        if self.tool_chain == 'hcc':
            # Check whether the env contains hcc.
            compiler_path = distutils.spawn\
                            .find_executable("riscv32-linux-musl-gcc")
            if compiler_path is None:
                # If no, set the hcc path to the env.
                compiler_path = pathlib.Path(tools_path)\
                           .joinpath(cur_sys, hcc_name, 'bin')
        elif self.tool_chain == 'hcc_fpu':
            # Check whether the env contains hcc_fpu.
            compiler_path = distutils.spawn\
                            .find_executable("riscv32-linux-musl-gcc")
            if compiler_path is None:
                # If no, set the hcc_fpu path to the env.
                compiler_path = pathlib.Path(tools_path)\
                           .joinpath(cur_sys, hccfpu_name, 'bin')
        else:
            raise Exception('Error: Unsupported compiler {}.'\
                            .format(self.tool_chain))

        str_path = 'PATH'
        if cur_sys == "Linux":
            # Setting Temporary Environment Variables
            os.environ[str_path] = "{}:{}:{}:{}".format(os.environ[str_path],
                                 gn_path, ninja_path, compiler_path)
        elif cur_sys == "Windows":
            # Setting Temporary Environment Variables
            os.environ[str_path] = "{};{};{};{}".format(os.environ[str_path],
                                 gn_path, ninja_path, compiler_path)

    # get compile cmd
    def get_cmd(self, gn_path, ninja_path):
        if not pathlib.Path(self.config).exists():
            raise Exception('Error: {} not exist, please check.'.format(
                            self.config))
        return self.__parse_compile_config(gn_path, ninja_path)

    def get_gn_args(self):
        self.args_list.append(self.cfg.get('gn_args', 'build_type_args'))
        self.args_list.append(self.cfg.get('gn_args', 'toolchain_args'))
        return "".join(self.args_list).replace('\"', '\\"')

    def __set_path(self):
        self.__root_path = pathlib.Path.cwd()
        self.__build_path = pathlib.Path(self.__root_path).joinpath('build')
        if not pathlib.Path(self.__build_path).exists():
            raise Exception('Error: {} not exist, please check.'.format(
                            self.__build_path))
        self.__out_path = pathlib.Path(self.__root_path)\
                          .joinpath('out')

    def __parse_compile_config(self, gn_path, ninja_path):
        section = 'env'
        self.cfg.set(section, 'build_path', str(self.get_build_path()))
        out_relpath = os.path.relpath(self.get_out_path())
        self.cfg.set(section, 'out_path', str(out_relpath))
        self.cfg.set(section, 'gn_path', gn_path)
        self.cfg.set(section, 'ninja_path', ninja_path)
        self.cfg.set(section, 'gn_args', self.get_gn_args())
        return [self.cfg.get(section, 'gn_cmd'),
                self.cfg.get(section, 'ninja_cmd')]


class Compile():
    '''
    Function description: Obtain the path of the compilation tool and 
    start compilation.
    '''

    gn_path = None
    ninja_path = None

    @classmethod
    def get_tool_path(cls):
        # Check whether the GN file exists.
        cls.gn_path = distutils.spawn.find_executable('gn')
        if cls.gn_path is None:
            raise Exception('Error: Can\'t find gn, install it please.')

        # Check whether the NINJA file exists.
        cls.ninja_path = distutils.spawn.find_executable('ninja')
        if cls.ninja_path is None:
            raise Exception('Error: Can\'t find ninja, install it please.')

    def compile(self, config):
        cmd_list = config.get_cmd(self.gn_path, self.ninja_path)
        for cmd in cmd_list:
            # Strings can be directly used in the Windows environment.
            if sys.platform == 'linux':
                cmd = shlex.split(cmd)
            # If shell is True, cmd is a string; if not, a sequence.
            exec_command(cmd, log_path=config.log_path, shell=False)


class CallbackDict(object):
    '''
    Function description: ???
    '''

    handlers = None

    # write the default value.
    def __init__(self):
        self.handlers = collections.defaultdict(list)

    def register(self, event, callback):
        self.handlers[event].append(callback)

    # ???
    def execute(self, event, **kwargs):
        if event not in self.handlers:
            raise Exception('{} not found in callback dict'.format(event))
        for handler in self.handlers.get(event, []):
            handler(**kwargs)


def main(argv):
    '''
    Function description: build and compile entry function.
    '''

    # Read the default command value
    configini_path = pathlib.Path.cwd().joinpath('build', 'config.ini')
    configini = ConfigParser()
    configini.read(configini_path)
    buildtype_default = configini.get('gn_args', 'build_type')
    toolchain_default = configini.get('gn_args', 'toolchain_select')

    # Command parser.
    parser = argparse.ArgumentParser(usage=usage(),
                                     description='auto build system')
    parser.add_argument('action', help='build or checkbuild or clean or info',
                        nargs='*', default=['info'])
    parser.add_argument('-b', '--build_type', help='release or debug version.',
                        nargs=1, default=['{}'.format(buildtype_default)])
    parser.add_argument('-t', '--tool_chain', help='hcc or hcc_fpu.',
                        nargs=1, default=['{}'.format(toolchain_default)])
    parser.set_defaults(command=exec_create)
    args = parser.parse_args()

    try:
        status = args.command(args)
    # Generally, press Ctrl+C to raise exception.
    except KeyboardInterrupt:
        logging.warning('interrupted')
        status = -1
    # Catch Other Exceptions
    except Exception as exce:
        parser.print_help()
        status = -1
    finally:
        pass

    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv))
