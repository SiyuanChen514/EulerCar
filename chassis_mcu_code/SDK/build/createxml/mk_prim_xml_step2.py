#!/usr/bin/env python
# coding=utf-8
# Purpose:
# Copyright Huawei Technologies Co.,Ltd. 2022-2022. All rights reserved
# Author:
 
import os
import time
import string
import re
import shutil
import time
import hashlib
import binascii
import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom as XDM


class CreateXml():
    '''
    Function description: create xml file
    '''
    
    def __init__(self):
        self.params = {}


    @staticmethod
    def get_loglevel(prim_pri):
        prim_loglevel = 'UNKNOWN'
        if int(prim_pri) == 0:
            real_pri = int(prim_pri)
            prim_loglevel = 'FATAL'
        elif int(prim_pri) == 1:
            real_pri = int(prim_pri)
            prim_loglevel = 'ERROR'
        elif int(prim_pri) == 2:
            real_pri = int(prim_pri)
            prim_loglevel = 'WARNING'
        elif int(prim_pri) == 3:
            real_pri = int(0)
            prim_loglevel = 'INFO'
        elif int(prim_pri) == 4:
            real_pri = int(0)
            prim_loglevel = 'DEBUG'
        return prim_loglevel


    @staticmethod
    def add_msg_to_xml(file_hdlr, contents):
        msg_hdlr = ET.Element('Msg')
        attributes = {}
        attributes['id'] = str(contents['prim_id'])
        attributes['loglevel'] = contents['prim_loglevel']
        attributes['description'] = contents['prim_msg']
        msg_hdlr.attrib = attributes
        file_hdlr.append(msg_hdlr)


    def get_cfg_files(self, build_xml_para):
        self.params = build_xml_para
        cfg_dir = self.params.get('prim_xml_cfg_dir')
        files = os.listdir(cfg_dir)
        cfg_files = []
        for file_name in files:
            if file_name[-4:] == '.cfg':
                cfg_files.append(os.path.join(cfg_dir, file_name))
        self.params['CFG_FILES'] = cfg_files
 
 
    def init_tree(self):
        xml_string = '<?xml version="1.0" encoding="utf-8"?><Log></Log>'
        root = ET.fromstring(xml_string)
        tree = ET.ElementTree(root)
        self.params['XML_ROOT'] = root
 
 
    def add_module_to_xml(self, root_hdlr, contents):
        prim_mod_id = contents['prim_mod_id']
    
        module_hdlr = ET.Element('Module')
        attributes = {}
        attributes['name'] = prim_mod_id
        module_hdlr.attrib = attributes
        root_hdlr.append(module_hdlr)
    
        self.params[prim_mod_id] = {}
        self.params.get(prim_mod_id)['handler'] = module_hdlr
        return module_hdlr
 
 
    def add_file_to_xml(self, module_hdlr, contents):
        prim_mod_id = contents['prim_mod_id']
        prim_file_id = contents['prim_file_id']
    
        file_hdlr = ET.Element('File')
        attributes = {}
        attributes['id'] = str(prim_file_id)
        file_hdlr.attrib = attributes
        module_hdlr.append(file_hdlr)
    
        self.params.get(prim_mod_id)[prim_file_id] = {}
        self.params.get(prim_mod_id).get(prim_file_id)['handler'] = file_hdlr
        return file_hdlr
 
 
    def add_contents_to_xml(self, contents):
        prim_file_id = contents['prim_file_id']
        prim_mod_id = contents['prim_mod_id']
        if prim_mod_id not in self.params.keys():
            root_hdlr = self.params.get('XML_ROOT')
            module_hdlr = CreateXml.add_module_to_xml(self, root_hdlr, contents)
            file_hdlr = CreateXml.add_file_to_xml(self, module_hdlr, contents)
            CreateXml.add_msg_to_xml(file_hdlr, contents)
        elif prim_file_id not in self.params.get(prim_mod_id):
            module_hdlr = self.params.get(prim_mod_id).get('handler')
            file_hdlr = CreateXml.add_file_to_xml(self, module_hdlr, contents)
            CreateXml.add_msg_to_xml(file_hdlr, contents)
        else:
            file_hdlr = self.params.get(prim_mod_id).get(prim_file_id).get(
                'handler')
            CreateXml.add_msg_to_xml(file_hdlr, contents)
 
 
    def parse_cfg_file_singleton(self, cfg_file):
        with open(cfg_file, encoding='utf-8') as fp:
            for line in fp:
                contents = {}
                match_pri = re.search('_PRIM_PRI_ = ', line)
                match_msg = re.search(', _PRIM_MSG_ = ', line)
                match_line = re.search(', _PRIM_LINE_ = ', line)
                match_file_id = re.search(', _PRIM_FILE_ID_ = ', line)
                match_mod_id = re.search(', _PRIM_MOD_ID_ = ', line)
                match_end = re.search(', _PRIM_END_', line)
    
                prim_pri = line[match_pri.end():match_msg.start()]
                prim_msg = line[match_msg.end():match_line.start()].strip(r'"')
                prim_line = line[match_line.end():match_file_id.start()]
                prim_file_id = line[match_file_id.end():match_mod_id.start()]
                prim_mod_id = line[match_mod_id.end():match_end.start()]
                contents['prim_id'] = (int(prim_line)) | (int(prim_file_id) <<
                    16)
                contents['prim_loglevel'] = CreateXml.get_loglevel(prim_pri)
                contents['prim_msg'] = prim_msg
                contents['prim_file_id'] = prim_file_id
                contents['prim_mod_id'] = prim_mod_id
    
                CreateXml.add_contents_to_xml(self, contents)
 
 
    def parse_cfg_file(self):
        for cfg_file in self.params.get('CFG_FILES'):
            CreateXml.parse_cfg_file_singleton(self, cfg_file)
 
 
    def tree_to_xml(self):
        root = self.params.get('XML_ROOT')
        xml_string = ET.tostring(root, encoding='utf-8')
        flags = os.O_RDWR | os.O_CREAT
        xdm = XDM.parseString(xml_string)
        xml_path = self.params.get('prim_xml_dst_full_path')
        with os.fdopen(os.open(xml_path, flags, 0o755), 'wb') as f:
            f.write(xdm.toprettyxml(indent='    ', encoding='utf-8'))
 
 
def copy_dir():
    root_path = os.getcwd()
    splicing_path = "drivers/debug/log/inc/ext_file_id_defs.h"
    src_path = os.path.join(root_path, splicing_path)
    target_path = os.path.dirname(self.params.get('PRIM_XML_DST_FULL_PATH'))
    shutil.copy(src_path, target_path)
 
 
def main():
    createxml = CreateXml()
    createxml.get_cfg_files()
    createxml.init_tree()
    createxml.parse_cfg_file()
    createxml.tree_to_xml()
    copy_dir()
 
if __name__ == '__main__':
    main()