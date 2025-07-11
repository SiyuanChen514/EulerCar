#!/usr/bin/env python
# coding=utf-8
# Purpose:
# Copyright Huawei Technologies Co.,Ltd. 2022-2022. All rights reserved
# Author:
 
import json
import os
import re
import shutil
import sys
import subprocess
import time
import stat
 

class CreateCfg():
    '''
    Function description: create config file.
    '''

    def __init__(self):
        self.params = {}
        self.file_id_dic = {}
        self.dst_full_file_name_list = []
        self.last_file_id_num = 0


    @staticmethod
    def get_file_id_str(src_file_name):
        if not os.path.isfile(src_file_name):
            return -1
        with open(src_file_name, 'r', encoding='utf-8',
            errors='replace') as file:
            for line in file:
                mod = re.search(\
                    "^#define[\s]+THIS_FILE_ID[\s]+(FILE_ID_[\w]*)",\
                    line.strip())
                if mod is None:
                    continue
                return mod.groups()[0]


    @staticmethod
    def get_necessary_information_singleton(fp, file_name):
        '''
        Function description: get necessary information for prim xml
        '''
        flags = os.O_RDWR | os.O_CREAT
        modes = stat.S_IWUSR | stat.S_IRUSR
        with os.fdopen(os.open(file_name, flags, modes), 'r+') as src_fp:
            line = src_fp.readline()
            while line:
                match = re.search('_PRIM_ST', line)
                if match:
                    fp.write(line)
                line = src_fp.readline()


    def save_file_id_dict(self, line):
        file_id_str = ''
        file_id_num = 0
    
        mod_1 = re.search("^(FILE_ID_[\w]*).+=\s+(\d*)", line.strip())
        mod_2 = re.search("^(FILE_ID_[\w]*)", line.strip())
        if mod_1 is not None:
            file_id_str = mod_1.groups()[0]
            file_id_num = mod_1.groups()[1]
            self.file_id_dic[file_id_str] = int(file_id_num)
            self.last_file_id_num = int(file_id_num)
        elif mod_2 is not None:
            file_id_str = mod_2.group()
            self.last_file_id_num += 1
            self.file_id_dic[file_id_str] = self.last_file_id_num
        else:
            return -1
        return 0
 
 
    def create_file_id_dic(self):
        file_name = self.params.get('hdb_xml_file_id', 'Not exist')
        if not os.path.exists(file_name):
            return
    
        file_d = open(file_name, 'r')
        lines = file_d.readlines()
        file_d.close()
        fsm_status = 0
        for line in lines:
            if 0 == fsm_status:
                mod = re.search("^typedef enum {", line.strip())
                if mod is not None:
                    fsm_status = 1
            elif 1 == fsm_status:
                mod_1 = re.search("^FILE_ID_[\w]*", line.strip())
                mod_2 = re.search("^}", line.strip())
                if mod_1 is not None:
                    CreateCfg.save_file_id_dict(self, line.strip())
                elif mod_2 is not None:
                    fsm_status = 2
            elif 2 == fsm_status:
                break


    def conver_c_2_i(self, full_file_name_list):
        not_exist = 'Not exist'
        temp_dir = self.params.get('i_file_dir', not_exist)
        if os.path.isdir(temp_dir):
            shutil.rmtree(temp_dir)
        os.makedirs(temp_dir)
        cflag = self.params.get('cflags', not_exist)
        include = self.params.get('include', not_exist)
        for src_full_file_name in full_file_name_list:
            file_id_str = CreateCfg.get_file_id_str(src_full_file_name)
            if file_id_str is None:
                continue
            src_file_name = src_full_file_name.rsplit(os.path.sep, 1)[-1]
            dst_file_name = src_file_name
            dst_file_name = dst_file_name.replace('.c', '.i')
            dst_full_file_name = os.path.join(temp_dir, dst_file_name)
            file_id_str = self.file_id_dic.get(file_id_str)
            if file_id_str is None:
                raise Exception(self.file_id_dic)
            cmd_line = ['riscv32-linux-musl-gcc', '-E', src_full_file_name] + \
                    cflag + include + ["-DMAKE_PRIM_XML_PROCESS_IN", 
                                        '-D__FILE_NAME__ = %s' % src_file_name, 
                                        '-D__FILE_IDX__ = %s' % file_id_str, 
                                        '-P', '-o', dst_full_file_name]
            subprocess.run(cmd_line, check=True)
            self.dst_full_file_name_list.append(dst_full_file_name)
        return self.dst_full_file_name_list
 
 
    def generate_params_dic(self, build_xml_para):
        '''
        Function description: convert input to dictionary
        '''
        self.params = build_xml_para
        CreateCfg.create_file_id_dic(self)
        full_file_name_list = self.params.get('sources')
        self.dst_full_file_name_list = CreateCfg.conver_c_2_i(self,
            full_file_name_list)
        if self.dst_full_file_name_list is None:
            return -1
 
 
    def get_necessary_information(self):
        '''
        Function description: store necessary information to .cfg
        '''

        cfg_file = self.params.get('prim_xml_cfg_file')
        flags = os.O_RDWR | os.O_CREAT
        modes = stat.S_IWUSR | stat.S_IRUSR
        if os.path.isfile(cfg_file):
            os.remove(cfg_file)
        if os.path.exists(os.path.dirname(cfg_file)) is False:
            os.makedirs(os.path.dirname(cfg_file))
        with os.fdopen(os.open(cfg_file, flags, modes), 'w+') as cfg_fp:
            for src_file in self.dst_full_file_name_list:
                CreateCfg.get_necessary_information_singleton(
                    cfg_fp, src_file)


def main():
    createcfg = CreateCfg()
    createcfg.generate_params_dic()
    createcfg.get_necessary_information()
 
if __name__ == '__main__':
    main()