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
# build_gn.py Function implementation: Automatically builds the compilation 
# framework.

import sys
import os
import stat
import subprocess
import pathlib
import json
import shlex
import shutil

from createxml.mk_prim_xml_step1 import CreateCfg
from createxml.mk_prim_xml_step2 import CreateXml
 

def read_json_file(input_file):
    '''
    Function description: Read the json file.
    '''

    if not pathlib.Path(input_file).exists():
        raise Exception('file [{}] no exist.'.format(input_file))
    data = None
    with open(input_file, 'rb') as input_f:
        data = json.load(input_f)
    return data


def del_allgn():
    '''
    Function description: Delete all GN build scripts.
    '''

    for (dirpath, _, filenames) in os.walk(pathlib.Path()):
        if "build" in dirpath or\
           ("middleware" in dirpath and "hisilicon" in dirpath):
            continue
        for file in filenames:
            if str(pathlib.Path(file)) != 'BUILD.gn':
                continue
            os.remove(pathlib.Path(dirpath).joinpath(file))
    global_buildfile = pathlib.Path().joinpath('build', 'BUILD.gn')
    if global_buildfile.exists():
        os.remove(global_buildfile)


class AutoCreate():
    '''
    Function description: Automatically builds the compilation framework.
    '''

    def __init__(self, json_content):
        '''
        Function description: Initialization is invoked by default.
        '''

        if 'system' not in json_content:
            raise Exception('Error: system not exist,please check.')

        # Stores information about the compile subsystem.
        compile_dict = dict()
        # Stores information about the transplant subsystem.
        transplant_dict = dict()
        # Save the path of third-party components so that 
        # they are not scanned during automatic construction.
        self.ext_component_path = []

        for subsystem in json_content['system']:
            if subsystem['name'] == 'compile':
                compile_dict = subsystem
            elif subsystem['name'] == 'transplant':
                transplant_dict = subsystem

        if transplant_dict:
            self.subsystem_transplant(transplant_dict)
        if compile_dict:
            self.subsystem_compile(compile_dict)
        else:
            raise Exception('Error: compile subsystem not exist,please check.')

    @staticmethod
    def check_extcomponentkeys_isexists(ext_component):
        # Instantiation parameter check.
        str_name = 'name'
        str_ext_component = 'ext_component'
        str_exec_path = 'exec_path'
        if not isinstance(ext_component, dict):
            raise TypeError("ext_component in para type error {}".format(
                            type(ext_component)))
        is_exception = str_name not in ext_component or\
           str_ext_component not in ext_component or\
           not ext_component.get(str_name) or\
           not ext_component.get(str_ext_component)
        if is_exception:
            raise Exception('Error: ext_components are incomplete,'
                            'Identification failed.')

        # Check whether the key exists.
        if str_exec_path not in ext_component[str_ext_component]:
            raise Exception('Error: {} key exec_path not exist, please'
                            'check.'.format(ext_component.get(str_name)))
        elif 'exec_cmd' not in ext_component[str_ext_component]:
            raise Exception('Error: {} key exec_cmd not exist, please'
                            'check.'.format(ext_component.get(str_name)))
        elif 'includes' not in ext_component[str_ext_component]:
            raise Exception('Error: {} key includes not exist, please'
                            'check.'.format(ext_component.get(str_name)))

        # If the value is empty, an exception is thrown.
        if not ext_component[str_ext_component][str_exec_path]:
            raise Exception('Error: {} value exec_path not exist, please'
                                'check.'.format(ext_component.get(str_name)))
        if not ext_component[str_ext_component]['exec_cmd']:
            raise Exception('Error: {} value exec_cmd not exist, please'
                                'check.'.format(ext_component.get(str_name)))
        # An exception is thrown when the sources fails to be searched.
        if not pathlib.Path(ext_component[str_ext_component]
                            [str_exec_path]).is_dir():
            raise Exception('Error: {} dir not exist, please '
                            'check.'.format(ext_component.get(str_name)))

    @staticmethod
    def copy_ext_component_includes(includes_list):
        # Instantiation parameter check.
        if not isinstance(includes_list, list):
            raise TypeError("includes_list in para type error {}".format(
                            type(includes_list)))

        for include in includes_list:
            if not pathlib.Path(include).is_dir():
                raise Exception('Error: {} is not a dir, please '
                            'check.'.format(include)) 
            for (dirpath, _, filenames) in os.walk(pathlib.Path(
                                                          include)):
                for file in filenames:
                    if pathlib.Path(file).suffix != '.h':
                        continue
                    include_file = pathlib.Path(dirpath).joinpath(file)
                    copy_path = pathlib.Path('middleware').joinpath(
                                'thirdparty', 'sysroot', 'include')
                    shutil.copy(include_file, copy_path)

    @staticmethod
    def cmd_exec(command):
        '''
        Function description: Run the compilation command.
        '''

        # Instantiation parameter check.
        if not isinstance(command, str):
            raise TypeError("command in para type error {}".format(
                            type(command)))

        cmd = shlex.split(command)
        proc = subprocess.Popen(cmd, shell=False)
        proc.wait()
        ret_code = proc.returncode
        if ret_code != 0:
            raise Exception("{} failed, return code is {}".format(cmd,
                                                                  ret_code))

    @staticmethod
    def check_modulekeys_isexists(module_content):
        # Check whether the key exists.
        str_sources = 'sources'
        if 'name' not in module_content:
            raise Exception('Error: {} key name not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'target_type' not in module_content:
            raise Exception('Error: {} key target_type not exist, please'
                            'check.'.format(module_content.get(str_sources)))
        elif 'includes' not in module_content:
            raise Exception('Error: {} key includes not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'define' not in module_content:
            raise Exception('Error: {} key define not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'libs' not in module_content:
            raise Exception('Error: {} key libs not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'lds_scripts' not in module_content:
            raise Exception('Error: {} key lds_scripts not exist, please'
                            'check.'.format(module_content.get(str_sources)))
        elif 'cflags' not in module_content:
            raise Exception('Error: {} key cflags not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'asmflags' not in module_content:
            raise Exception('Error: {} key asmflags not exist, please check.'
                            .format(module_content.get(str_sources)))
        elif 'ldflags' not in module_content:
            raise Exception('Error: {} key ldflags not exist, please check.'
                            .format(module_content.get(str_sources)))

    @staticmethod
    def nochecklist(path, nocheck_list):
        '''
        Function description: If the path exists in the nocheck_list,
        a false is returned,Otherwise, a true is returned.
        '''

        # Instantiation parameter check.
        if not isinstance(path, str):
            raise TypeError("path in para type error {}".format(
                            type(path)))
        if not isinstance(nocheck_list, list):
            raise TypeError("nocheck_list in para type error {}".format(
                            type(nocheck_list)))

        for nocheck in nocheck_list:
            if nocheck in path:
                return False

        return True

    def subsystem_transplant(self, subsystem):
        '''
        Function description: transplant Subsystem
        '''

        # Instantiation parameter check.
        if not isinstance(subsystem, dict):
            raise TypeError("subsystem in para type error {}".format(
                            type(subsystem)))
        
        ext_component_key = 'ext_component'
        for ext_component in subsystem['subsystem']:
            if not ext_component:
                continue
            self.check_extcomponentkeys_isexists(ext_component)
            
            path = str(pathlib.Path(
                       ext_component[ext_component_key]['exec_path']))
            self.ext_component_path.append(path)
            command = ext_component[ext_component_key]['exec_cmd']
            if ext_component[ext_component_key]['includes']:
                self.copy_ext_component_includes(ext_component[ext_component_key]
                                                 ['includes'])

            curr_dir = os.getcwd()
            os.chdir(path)
            if '&&' in command:
                command_list = command.split('&&')
                for cmd in command_list:
                    self.cmd_exec(cmd)
            else:
                self.cmd_exec(command)
            os.chdir(os.path.realpath(curr_dir))

    def subsystem_compile(self, subsystem):
        '''
        Function description: Compiling Subsystem
        '''

        # Instantiation parameter check.
        if not isinstance(subsystem, dict):
            raise TypeError("subsystem in para type error {}".format(
                            type(subsystem)))

        # Records the list of modules to be built in the JSON file.
        self.json_module_name = []
        self.json_module_path = []
        # List of paths that are not checked.
        self.nocheck = []
        # Save global build parameters in a dictionary.
        globalbuild_dict = dict()

        # Read the JSON file to build the module compilation script.
        for component in subsystem['subsystem']:
            # Perform global build after module build is complete.
            if component['name'] == 'compile_frame':
                globalbuild_dict = component
                continue
            for module in component['component']:
                # If the key exists and the key value also exists
                if 'sources' in module and\
                   module.get('sources'):
                    self.localgn_create(module)

        if not globalbuild_dict:
            raise Exception('Error: Global Build Parameters not exist,'
                            'please check.')
        # Global compilation script building
        self.globalgn_create(globalbuild_dict)

    def localgn_create(self, module_content):
        '''
        Function description: Module partial construction.
        notes: 
        (1)Create the Build.gn file in the first path of sources.
        '''

        # Instantiation parameter check.
        if not isinstance(module_content, dict):
            raise TypeError("module_content in para type error {}".format(
                            type(module_content)))

        self.check_modulekeys_isexists(module_content)

        # Reads the content of the JSON file.
        # If the name value is empty, an exception is thrown.
        if module_content.get('name'):
            self.name = module_content['name']
        else:
            raise Exception('Error: module name is None, please check.')
        self.json_module_name.append(self.name)

        # If the target_type value is empty, an warning is thrown.
        if module_content.get('target_type'):
            self.target_type = module_content['target_type']
        else:
            self.target_type = "obj"

        # Converting a Relative Path to an Absolute Path.
        self.sources = []
        for source_path in module_content['sources']:
            self.sources.append("{}".format(
                                os.path.relpath(pathlib.Path(source_path))))
        self.json_module_path.append(self.sources)
        self.includes = []
        for include_path in module_content['includes']:
            self.includes.append("{}".format(
                                 os.path.relpath(pathlib.Path(include_path))))

        self.define = module_content['define']
        self.libs = module_content['libs']
        self.lds_scripts = module_content['lds_scripts']
        self.cflags = module_content['cflags']
        self.asmflags = module_content['asmflags']
        self.ldflags = module_content['ldflags']

        # Create the Build.gn file in the first path of sources.
        self.gn_create(self.sources[0])

    def globalgn_create(self, globalbuild_dict):
        '''
        Function description: Global Build Generates Executable Files.
        notes:
        (1)Create the Build.gn file in the build directory.
        (2)Executable file name:target.
        '''

        str_cflags = 'cflags'
        str_ldflags = 'ldflags'
        # Instantiation parameter check.
        if not isinstance(globalbuild_dict, dict):
            raise TypeError("globalbuild_dict in para type error {}".format(
                            type(globalbuild_dict)))

        # Check whether the key exists.
        if 'define' not in globalbuild_dict:
            raise Exception('Error: compile_frame key define not exist, please check.')
        elif str_cflags not in globalbuild_dict:
            raise Exception('Error: compile_frame key cflags not exist, please check.')
        elif 'asmflags' not in globalbuild_dict:
            raise Exception('Error: compile_frame key asmflags not exist, please check.')
        elif str_ldflags not in globalbuild_dict:
            raise Exception('Error: compile_frame key ldflags not exist, please check.')
        elif 'nocheck' not in globalbuild_dict:
            raise Exception('Error: compile_frame key nocheck not exist, please check.')

        # Check whether the value exists.
        if not globalbuild_dict.get(str_cflags):
            raise Exception('Error: global cflags is None, please check.')
        elif not globalbuild_dict.get(str_ldflags):
            raise Exception('Error: global ldflags is None, please check.')

        self.name = 'target.elf'
        self.target_type = 'executable'
        self.sources = []
        self.includes = []
        self.libs = []
        self.extlibspath = []
        self.extlibsname = []
        self.extlibsinclude = []
        for name, path in zip(self.json_module_name, self.json_module_path):
            if pathlib.Path(path[0]).is_file():
                self.libs.append("{0}:{1}".format(pathlib.Path(path[0]).parent, name))
            else:
                self.libs.append("{0}:{1}".format(path[0], name))
        self.define = globalbuild_dict['define']
        self.cflags = globalbuild_dict[str_cflags]
        self.asmflags = globalbuild_dict['asmflags']
        self.ldflags = globalbuild_dict[str_ldflags]
        for nocheck_path in globalbuild_dict['nocheck']:
            self.nocheck.append("{}".format(pathlib.Path(nocheck_path)))
        if 'extlibspath' in globalbuild_dict:
            self.extlibspath = globalbuild_dict['extlibspath']
        if 'extlibsname' in globalbuild_dict:
            self.extlibsname = globalbuild_dict['extlibsname']
        if 'extlibsinclude' in globalbuild_dict:
            self.extlibsinclude = globalbuild_dict['extlibsinclude']

        self._build_path = pathlib.Path.cwd().joinpath('build')
        self.gn_create(self._build_path)

    def gn_create(self, path):
        '''
        Function description: Creating a BUILD.gn File.
        '''

        # Module Building Content List
        self.build_content = []
        if pathlib.Path(path).is_file():
            path = pathlib.Path(path).parent
        buildgn_path = pathlib.Path(path).joinpath('BUILD.gn')

        flags = os.O_WRONLY | os.O_CREAT
        modes = stat.S_IWUSR | stat.S_IRUSR
        with os.fdopen(os.open(buildgn_path, flags, modes),
                       'w') as build_file:
            self.build_content.append("import(\"//build/toolchain/config.gni\""
                                      ")\n")
            self.defaults_config()
            self.target_config()
            build_file.writelines(self.build_content)

    def defaults_config(self):
        '''
        Function description: default configuration of the target_type.
        '''

        if self.target_type == "obj":
            self.build_content.append("set_defaults(\"source_set\") {\n")
        elif self.target_type == "static":
            self.build_content.append("set_defaults(\"static_library\") {\n")
        elif self.target_type == "share":
            self.build_content.append("set_defaults(\"shared_library\") {\n")
        elif self.target_type == "executable":
            self.build_content.append("set_defaults(\"executable\") {\n")
        else:
            raise Exception('Error: {} is incorrect, please check.'.format(
                            self.target_type))

        self.compileflag_config()
        self.define_config()
        
        self.build_content.append('''    if(build_type == "debug") {
        cflags += [
            "-g"
        ]
    }else if(build_type == "release") {
        cflags += [
            "-fomit-frame-pointer"
        ]
        defines += [
            "NDEBUG"
        ]
    }
}''')
            
    def compileflag_config(self):
        '''
        Function description: Setting Compilation Parameters. 
        '''

        close_bracket = "    ]\n"
        prefix = "        \""
        suffix = "\",\n"
        # set cflags
        if self.cflags:
            self.build_content.append("    cflags = [\n")
            for cflags in self.cflags:
                self.build_content.append(prefix + cflags + suffix)
            self.build_content.append(close_bracket)

        # set asmflags
        if self.asmflags:
            self.build_content.append("    asmflags = [\n")
            for asmflags in self.asmflags:
                self.build_content.append(prefix + asmflags + suffix)
            self.build_content.append(close_bracket)

        # set ldflags
        if self.ldflags:
            self.build_content.append("    ldflags = [\n")
            for ldflags in self.ldflags:
                self.build_content.append(prefix + ldflags + suffix)

            self.lds_scripts_config()
            self.library_link_config()

            self.build_content.append(close_bracket)

    def define_config(self):
        '''
        Function description: Setting Precompiled Macros.
        '''

        # set define
        self.build_content.append("    defines = [\n")
        for define in self.define:
            self.build_content.append("        \"{}\",\n".format(define))
        self.build_content.append("    ]\n")

    def target_config(self):
        '''
        Function description: Required File Configuration for Target.
        '''

        if self.target_type == "obj":
            self.build_content.append("source_set(\"%s\") {\n"
                                      % (self.name))
        elif self.target_type == "static":
            self.build_content.append("static_library(\"%s\") {\n"
                                      % (self.name))
        elif self.target_type == "share":
            self.build_content.append("shared_library(\"%s\") {\n"
                                      % (self.name))
        elif self.target_type == "executable":
            self.build_content.append("executable(\"%s\") {\n"
                                      % (self.name))
        else:
            raise Exception('Error: {} is incorrect, please check.'.format(
                            self.target_type))

        self.sources_config()
        self.include_dirs_config()
        self.deps_config()

        self.build_content.append("}\n")
 
    def xml_create(self):
        build_xml_para = {}
        c_file = self.xmlfiles
        module_name = "mcu_xml"
        str_dfx_db = 'dfx_db'
        build_tmp_path = pathlib.Path.cwd().joinpath('build', 'createxml',
                                                     'mss_prim_db')
        if not build_tmp_path:
            os.makedirs(build_tmp_path)
        prim_xml_cfg_dir = build_tmp_path.joinpath(str_dfx_db, 'xml_cfg')
        prim_xml_cfg_file = build_tmp_path.joinpath(str_dfx_db, 'xml_cfg',
                                                    module_name + '.cfg')
        in_path = pathlib.Path.cwd().joinpath('build', 'createxml',
                                              'mk_dfx_xml.json')
        hdb_xml_temp_root_dir = build_tmp_path.joinpath('modules')
        hdb_xml_file_id = pathlib.Path.cwd().joinpath('drivers',
            'debug', 'log', 'inc', 'file_id_defs.h')
        prim_xml_key_word = module_name
        i_file_dir = build_tmp_path.joinpath('modules', module_name)
        prim_xml_dst_full_path = build_tmp_path.joinpath(str_dfx_db, 'log.xml')
        sources = c_file
        build_xml_para.update({"cflags": self.cflags})
        build_xml_para.update({"c_file": c_file})
        build_xml_para.update({"build_tmp_path": build_tmp_path})
        build_xml_para.update({"prim_xml_cfg_dir": prim_xml_cfg_dir})
        build_xml_para.update({"prim_xml_cfg_file": prim_xml_cfg_file})
        build_xml_para.update({"module_name": module_name})
        build_xml_para.update({"in_path": in_path})
        build_xml_para.update({"hdb_xml_temp_root_dir": hdb_xml_temp_root_dir})
        build_xml_para.update({"hdb_xml_file_id": hdb_xml_file_id})
        build_xml_para.update({"prim_xml_key_word": prim_xml_key_word})
        build_xml_para.update({"i_file_dir": i_file_dir})
        build_xml_para.update({"sources": sources})
        build_xml_para.update({"cc": "riscv32-linux-musl-gcc"})
        build_xml_para.update({"include": self.includes_path})
        build_xml_para.update({"prim_xml_dst_full_path":
                                   prim_xml_dst_full_path})
        createcfg = CreateCfg()
        createcfg.generate_params_dic(build_xml_para)
        createcfg.get_necessary_information()
        createxml = CreateXml()
        createxml.get_cfg_files(build_xml_para)
        createxml.init_tree()
        createxml.parse_cfg_file()
        createxml.tree_to_xml()
         
    def sources_config(self):
        '''
        Function description: Source File Configuration.
        '''

        if self.target_type != "executable":
            self.local_sources_scan()
            return True

        self.global_sources_scan()
        return True

    def global_sources_scan(self):
        '''
        Function description: Global Source File Search.
        '''

        self.nocheck_lists = []
        self.nocheck_lists_file = []

        self.build_content.append("    sources = [\n")
        self.xmlfiles = []
        for module_list in self.json_module_path:
            for sources_list in module_list:
                if pathlib.Path(sources_list).is_file():
                    self.nocheck_lists_file.append(sources_list)
                else:
                    self.nocheck_lists.append(sources_list)
        for nocheck_list in self.nocheck:
            self.nocheck_lists.append(nocheck_list)
        self.nocheck_lists.extend(self.ext_component_path)

        self._root_path = pathlib.Path()
        for (dirpath, dirnames, filenames) in os.walk(self._root_path):
            dirnames.sort()
            filenames.sort()
            # Find all module code except in the JSON file
            if not self.nochecklist(dirpath, self.nocheck_lists):
                continue

            for file_global in filenames:
                if pathlib.Path(file_global).suffix != '.c' and\
                   pathlib.Path(file_global).suffix != '.S':
                    continue
                if not self.nochecklist(str(pathlib.Path(dirpath).
                                        joinpath(file_global)),
                                        self.nocheck_lists_file):
                    continue
                if pathlib.Path(file_global).suffix == '.c':
                    self.xmlfiles.append("{}".format(
                                          pathlib.Path(dirpath)
                                          .joinpath(file_global)))
                self.build_content.append("        \"//{}\",\n".format(
                                          pathlib.Path(dirpath)
                                          .joinpath(file_global)))

        self.build_content.append("    ]\n")

    def local_subdirectory_sources_scan(self, sources, cfile_list):
        '''
        Function description: Search for source files in the subdirectory.
        '''

        for (dirpath, dirnames, filenames) in os.walk(sources):
            dirnames.sort()
            filenames.sort()
            for file_local in filenames:
                if (pathlib.Path(file_local).suffix != '.c' and\
                    pathlib.Path(file_local).suffix != '.S') or\
                    file_local in cfile_list:
                    continue
                cfile_list.append(file_local)
                self.build_content.append("        \"//{}\",\n"
                                            .format(pathlib.Path(dirpath)
                                            .joinpath(file_local)))

    def local_sources_scan(self):
        '''
        Function description: module Source File Search.
        '''

        cfile_list = []

        self.build_content.append("    sources = [\n")
        for sources in self.sources:
            # An exception is thrown when the sources fails to be searched.
            if pathlib.Path(sources).is_file():
                if pathlib.Path(sources).suffix == '.c' or\
                   pathlib.Path(sources).suffix == '.S':
                    self.build_content.append("        \"//{}\",\n"
                                              .format(sources))
            else:
                self.local_subdirectory_sources_scan(sources, cfile_list)

        self.build_content.append("    ]\n")

    def include_dirs_config(self):
        '''
        Function description: Include Dirs Configuration.
        '''

        # set includes
        if self.target_type != "executable":
            if self.includes:
                self.build_content.append("    include_dirs = [\n")
                for include in self.includes:
                    self.includes_scan(include)
                self.build_content.append("    ]\n")
            return True

        self.build_content.append("    include_dirs = [\n")
        self.includes_scan(pathlib.Path())
        for extlibsinclude in self.extlibsinclude:
            self.build_content.append("        \"//{}\",\n"
                                      .format(os.path.relpath(extlibsinclude)))
        self.build_content.append("    ]\n")  
        return True

    def includes_scan(self, path):
        self.includes_path = []
        for (dirpath, dirnames, filenames) in os.walk(path):
            dirnames.sort()
            filenames.sort()
            if not self.nochecklist(dirpath, self.ext_component_path):
                continue
            if not self.nochecklist(dirpath, self.nocheck):
                continue
            for file in filenames:
                if pathlib.Path(file).suffix != '.h':
                    continue
                self.build_content.append("        \"//{}\",\n"
                                          .format(pathlib.Path(dirpath)))
                self.includes_path.append("-I" + str(pathlib.Path.cwd()
                                                   .joinpath(dirpath)))
                break

    def deps_config(self):
        '''
        Function description: Dependency Library File Configuration.
        '''

        if self.target_type == "executable":
            if self.libs:
                self.build_content.append("    deps = [\n")

                for libs in self.libs:
                    self.build_content.append("        \"//{}\",\n"
                                              .format(libs))

                self.build_content.append("    ]\n")

    def lds_scripts_config(self):
        '''
        Function description: Lds File Configuration.
        '''

        if self.target_type == "executable":
            for filename in os.listdir(pathlib.Path("chip")):
                if filename == "target":
                    continue
                break
            self._lds_scripts_path = pathlib.Path("..").joinpath('chip',
                                     filename, 'flash.lds')
            self.build_content.append("        \"-T{}\",\n"
                                      .format(self._lds_scripts_path))

    def library_link_config(self):
        '''
        Function description: Static Library Link Configuration.
        '''

        if self.target_type != "executable":
            return

        libs_list = []
        out_path_name = 'out'
        libs_path_name = 'libs'
        spliter = '.'
        libs_path = ["../{}".format(pathlib.Path(out_path_name).joinpath(libs_path_name))]
        if pathlib.Path(out_path_name).joinpath(libs_path_name).exists():
            for libname in os.listdir(pathlib.Path(out_path_name).joinpath(libs_path_name)):
                if libname.split(spliter)[-1] != 'a':
                    continue
                libname = libname[3:-2]
                if libname not in self.json_module_name:
                    libs_list.append(libname)
        if not libs_list:
            libs_path = []

        for (dirpath, _, filenames) in os.walk(pathlib.Path(spliter)):
            if out_path_name in dirpath and libs_path_name in dirpath:
                continue
            for file in filenames:
                if file.split(spliter)[-1] == 'a':
                    libs_path.append(".{}".format(dirpath))
                    libs_list.append(file[3:-2])
        
        if libs_list or self.extlibsname:
            self.build_content.append("        \"-Wl,--whole-archive\",\n")
            # Deduplicate paths.
            libs_path = list(set(libs_path))
            for libpath in libs_path:
                self.build_content.append("        \"-L{}\",\n"
                                          .format(libpath))
            for extlibspath in self.extlibspath:
                self.build_content.append("        \"-L{}\",\n"
                                          .format(extlibspath))
            for liblist in libs_list:
                self.build_content.append("        \"-l{}\",\n"
                                          .format(liblist))
            for extlibsname in self.extlibsname:
                self.build_content.append("        \"-l{}\",\n"
                                          .format(extlibsname[3:-2]))
            self.build_content.append("        \"-Wl,--no-whole-archive\",\n")


def main(argv):
    '''
    Function description: buildgn entry function.
    '''

    #clear gn
    del_allgn()
    #auto build
    product_json = pathlib.Path.cwd().joinpath('chip', 'target',
                                               'userconfig.json')
    json_content = read_json_file(product_json)
    AutoCreate(json_content)


if __name__ == "__main__":
    sys.exit(main(sys.argv))