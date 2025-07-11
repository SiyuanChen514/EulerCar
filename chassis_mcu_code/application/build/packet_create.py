#!/usr/bin/env python3
# coding=utf-8

'''
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
'''
import struct
import sys
import os
import stat
import pathlib
import zlib
import copy
import socket
import collections
from configparser import ConfigParser


class Crc16:
    POLYNOMIAL = 0x1021
    PRESET = 0x0000
    _tab = []

    def __init__(self):
        self._tab = [self._initial(i) for i in range(256)]

    def crc(self, string):
        crc = self.PRESET
        for c in string:
            crc = self._update_crc(crc, ord(c))
        return crc

    def crcb(self, i):
        crc = self.PRESET
        for c in i:
            crc = self._update_crc(crc, c)
        return crc

    def _initial(self, c):
        crc = 0
        c = c << 8
        for _i in range(8):
            if (crc ^ c) & 0x8000:
                crc = (crc << 1) ^ self.POLYNOMIAL
            else:
                crc = crc << 1
            c = c << 1
        return crc

    def _update_crc(self, crc, c):
        cc = 0xff & int(c)

        tmp = (crc >> 8) ^ cc
        crc = (crc << 8) ^ self._tab[tmp & 0xff]
        crc = crc & 0xffff
        return crc


def get_config(name):
    deveco_path = './.deveco/deveco.ini'
    env = "env:" + name
    config = ConfigParser()
    config.read(deveco_path)
    init_value = 'no'
    config_dict = dict(generate_crc=init_value, generate_checksum=init_value, padding=init_value)
    if name == init_value:
        return config_dict
    key = 'generate_crc'
    if key in config[env]:
        config_dict[key] = config[env].get(key)
    key = 'generate_checksum'
    if key in config[env]:
        config_dict[key] = config[env].get(key)
    key = 'padding'
    if key in config[env]:
        config_dict[key] = config[env].get(key)
    return config_dict


def packet_bin(output_path, input_list):
    t = Crc16()
    path_list = []
    burn_addr_list = []
    burn_size_list = []
    image_size_list = []
    type_list = []
    for item in input_list:
        path, burn_addr, burn_size, burn_type = item.split("|")
        image_size = os.path.getsize(path)
        path_list.append(path)
        burn_addr_list.append(int(burn_addr))
        burn_size_list.append(int(burn_size))
        image_size_list.append(image_size)
        type_list.append(int(burn_type))

    flag = 0xefbeaddf
    crc = 0
    image_num = len(path_list)
    head_len = image_num * 52 + 12
    total_file_size = sum(image_size_list) + head_len

    flags = os.O_RDWR | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(output_path, flags, modes), 'wb+') as file:
        file.write(struct.pack('IHHI', flag, crc, image_num, total_file_size))
        start_index = head_len
        times = 0
        for path in path_list:
            path_name = os.path.basename(path)
            file.write(
                struct.pack('32sIIIII', bytes(path_name, 'ascii'), start_index,
                            image_size_list[times], burn_addr_list[times],
                            burn_size_list[times], type_list[times]))
            start_index = start_index + image_size_list[times] + 16
            times += 1

        for path in path_list:
            with os.fdopen(os.open(path, flags, modes), 'rb+') as subfile:
                data = subfile.read()
                file.write(data)
                file.write(struct.pack('IIII', 0, 0, 0, 0))

        file.flush()
        file.seek(6)
        newdata = file.read(head_len - 6)
        crc16 = t.crcb(newdata)
        file.seek(4)
        file.write(struct.pack('H', crc16))


def get_len_addr_type(line):
    data = int(line[1:9], 16)
    length = data >> 24
    addr = (data >> 8) & 0xffff
    data_type = data & 0xff
    ext_data = int(line[10:13], 16)
    Information = collections.namedtuple('Information', ['length', 'addr', 'data_type', 'ext_data'])
    return Information(length, addr, data_type, ext_data)


def is_start_linear_addr_rec_line(data_type):
    return True if data_type == 5 else False


def is_ext_linear_addr_rec_line(data_type):
    return True if data_type == 4 else False


def add_ext_linear_addr_record(fp, addr):
    checksum = 0
    data = []
    data.append(':02000004')
    checksum += 6
    checksum += (addr & 0xff)
    checksum += ((addr >> 8) & 0xff)
    data.append("%04x".upper() % (addr))
    checksum = (0x100 - checksum % 0x100) & 0xFF
    data.append("%02x".upper() % (checksum))
    data.append('\n')
    fp.writelines(data)


def add_data_of_crc(fp, addr, crc_val):
    checksum = 0
    data = []
    data.append(':04')
    checksum += 4
    data.append("%04x".upper() % (addr))
    checksum += (addr & 0xff)
    checksum += ((addr >> 8) & 0xff)
    data.append('00')
    data.append("%08x".upper() % (socket.htonl(crc_val)))
    checksum += (crc_val & 0xff)
    checksum += ((crc_val >> 8) & 0xff)
    checksum += ((crc_val >> 16) & 0xff)
    checksum += ((crc_val >> 24) & 0xff)
    checksum = (0x100 - checksum % 0x100) & 0xFF
    data.append("%02x".upper() % (checksum))
    data.append('\n')
    fp.writelines(data)


def add_crc(fp, ext_addr, address, crc_val):
    addr = address
    if addr > 0xFFFF:
        add_ext_linear_addr_record(fp, ext_addr + 1)
        addr = 0
    add_data_of_crc(fp, addr, crc_val)


def gen_crc_padding(fp, lines, crc_val):
    is_start_linear_addr_rec = False
    ext_linear_addr = 0
    next_addr = 0

    for line in lines:
        length, addr, data_type, ext_data = get_len_addr_type(line)
        if is_ext_linear_addr_rec_line(data_type):
            ext_linear_addr = ext_data
        if not is_start_linear_addr_rec_line(data_type):
            next_addr = addr + length
        else:
            if not is_start_linear_addr_rec:
                is_start_linear_addr_rec = True
                add_crc(fp, ext_linear_addr, next_addr, crc_val)
        fp.writelines(line)


def gen_crc_for_hex(filename, crc_val):
    flags = os.O_RDWR
    modes = stat.S_IWUSR | stat.S_IRUSR

    lines = []
    with os.fdopen(os.open(filename, flags, modes), 'r') as file:
        lines = file.readlines()

    flag = os.O_RDWR | os.O_CREAT
    with os.fdopen(os.open(filename, flag, modes), 'w+') as file:
        gen_crc_padding(file, lines, crc_val)


def gen_crc(filename, filename_hex):
    flags = os.O_RDWR
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(filename, flags, modes), 'rb') as f:
        data = f.read()
        crc_val = zlib.crc32(data) & 0xFFFFFFFF
    with os.fdopen(os.open(filename, flags, modes), 'rb+') as file:
        file.seek(0, 2)
        crc_val = socket.htonl(crc_val)
        file.write(struct.pack('I', crc_val))
        gen_crc_for_hex(filename_hex, crc_val)


def findfiles(path, types):
    file_list = []
    files = os.listdir(path)
    for f in files:
        npath = "{}/{}".format(path, f)
        if os.path.isfile(npath):
            if os.path.splitext(npath)[-1] in types:
                file_list.append(npath)
    return file_list


def packet_bin_with_padding(file, length, val):
    while length >= 32:
        file.write(struct.pack('IIIIIIII',
                               val, val, val, val, val, val, val, val))
        length = length - 32
    while length >= 4:
        file.write(struct.pack('I', val))
        length = length - 4
    while length > 0:
        file.write(struct.pack('B', val & 0xFF))
        length = length - 1


def gen_padding_for_bin(filename, max_len, pad_bit):
    flags = os.O_RDWR | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(filename, flags, modes), 'ab+') as file:
        pad_val = 0xFFFFFFFF
        if pad_bit == '0':
            pad_val = 0
        image_size = os.path.getsize(filename)
        pad_len = max_len - image_size
        packet_bin_with_padding(file, pad_len, pad_val)
        return pad_len


def get_hex_addr(lines):
    ext_addr = 0
    next_addr = 0
    for line in lines:
        length, addr, data_type, ext_data = get_len_addr_type(line)
        if data_type == 4:
            ext_addr = ext_data
        if data_type == 0:
            next_addr = addr + length
    return ext_addr, next_addr


def get_line_pad_len(addr, max_pad_len):
    line_space_size = 0xFFFF - addr + 1
    pad_len = 16 if max_pad_len >= 16 else max_pad_len
    if pad_len > line_space_size:
        pad_len = line_space_size
    return pad_len


def gen_ext_addr_rec(ext_addr):
    checksum = 0
    data = ''
    data += ':02000004'
    checksum += 6
    checksum += (ext_addr & 0xff)
    checksum += ((ext_addr >> 8) & 0xff)
    data += "%04x".upper() % ext_addr
    checksum = (0x100 - (checksum & 0xFF)) & 0xFF
    data += "%02x\n".upper() % (checksum)
    return data


def gen_padding_line_rec(addr, length, pad_bit):
    checksum = 0
    data = ''
    data += ":%02x".upper() % length
    checksum += length & 0xff
    checksum += (addr & 0xff)
    checksum += ((addr >> 8) & 0xff)
    data += "%04x00".upper() % addr
    pad_val = 0xFF if pad_bit == '1' else 0
    for _i in range(length):
        data += "%02x".upper() % pad_val
        checksum += pad_val
    checksum = (0x100 - (checksum & 0xFF)) & 0xFF
    data += "%02x\n".upper() % (checksum)
    return data


def gen_padding_lines_for_hex(ext_addr, addr, length, pad_bit):
    lines = []

    while length > 0:
        pad_len = get_line_pad_len(addr, length)
        lines.append(gen_padding_line_rec(addr, pad_len, pad_bit))
        length -= pad_len
        addr += pad_len
        if addr == 0x10000 and length > 0:
            ext_addr += 1
            lines.append(gen_ext_addr_rec(ext_addr))
            addr = 0
    return lines


def gen_padding_for_hex(filename, pad_len, pad_bit):
    '''
    Generate padding for target.hex
    '''
    flags = os.O_RDWR
    modes = stat.S_IWUSR | stat.S_IRUSR
    lines = []
    ext_addr = int(0)
    next_addr = int(0)
    with os.fdopen(os.open(filename, flags, modes), 'r') as file:
        lines = file.readlines()
        ext_addr, next_addr = get_hex_addr(lines)

    start_linear_addr_rec = lines[-2]
    end_of_file_rec = lines[-1]
    lines.pop()
    lines.pop()

    lines += gen_padding_lines_for_hex(ext_addr, next_addr, pad_len, pad_bit)
    lines += start_linear_addr_rec
    lines += end_of_file_rec

    flag = os.O_RDWR | os.O_CREAT
    with os.fdopen(os.open(filename, flag, modes), 'w+') as file:
        file.writelines(lines)


def gen_checksum_list(filename):
    '''
    Generate Checksum list
    '''
    path = "./out/bin"
    type_list = ['.bin', '.hex']
    file_list = findfiles(path, type_list)
    checksum_list = []

    flags = os.O_RDWR | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    for item in file_list:
        with os.fdopen(os.open(item, flags, modes), 'rb') as f:
            data = f.read()
            crc_val = zlib.crc32(data) & 0xFFFFFFFF
            file_crc_dict = {}
            file_crc_dict['file'] = os.path.split(item)[1]
            file_crc_dict['crc'] = hex(crc_val)
            checksum_list.append(file_crc_dict)

    with os.fdopen(os.open(filename, flags, modes), 'w') as f:
        for info in checksum_list:
            file_name = "{}\n".format(info['file'])
            f.write(file_name)
            crc_str = "CRC:{}\n\n".format(info['crc'])
            f.write(crc_str)


def main(argv):
    '''
    Function description:
    1. add crc for target.bin and hex.bin if generate_crc = yes
    2. add padding for target.bin if padding enable
    3. generate checksum_list.txt and print in IDE TERMINAL windows
    4. Combine loader.bin with image.bin into allinone.bin.
    '''
    chipname = 'no'
    flashsize = 0
    if len(argv) == 3:
        chipname = argv[1]
        flashsize = argv[2]
    cfg_dict = get_config(chipname)
    cfg_dict['max_len'] = flashsize

    loaderbin_path = "./middleware/hisilicon/loaderboot/loader.bin"
    targetbin_path = "./out/bin/target.bin"
    targethex_path = "./out/bin/target.hex"
    allinonebin_path = "./out/bin/allinone.bin"
    chksumlisttxt_path = "./out/bin/checksum_list.txt"
    str_padding = "padding"

    curpath = pathlib.Path().cwd()
    bootloaderpath = curpath.joinpath(loaderbin_path)
    eflashpath = curpath.joinpath(targetbin_path)
    output_path = curpath.joinpath(allinonebin_path)

    if cfg_dict.get('generate_crc') == 'yes':
        gen_crc(targetbin_path, targethex_path)

    if cfg_dict.get(str_padding) != 'no':
        pad_len = gen_padding_for_bin(targetbin_path,
                int(cfg_dict.get('max_len')),
                cfg_dict.get(str_padding))
        gen_padding_for_hex(targethex_path, pad_len, cfg_dict.get(str_padding))

    input_list = [
        "{}|{}|{}|0".format(
        bootloaderpath, 0x2000000, 0x2000000 + 0x3FFF),
        "{}|{}|{}|1".format(
        eflashpath, 0x3000000, 0x3000000 + 0x27FFF)
    ]

    packet_bin(output_path, input_list)

    if cfg_dict.get('generate_checksum') == 'yes':
        gen_checksum_list(chksumlisttxt_path)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
