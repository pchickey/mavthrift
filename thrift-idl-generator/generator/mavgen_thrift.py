#/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Copyright Pat Hickey 2013
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time, string
from thrift import *
import mavparse, mavtemplate, mavgen_process_xml

t = mavtemplate.MAVTemplate()

def thrift_name(field , m):
    f = field.name.encode('ascii')
    # catch reserved words
    if (f == "type" and m.name == "HEARTBEAT"):
        return "mavtype"
    if (f in ["type"]):
       return m.name.lower() + "_" + f
    else:
      return f

def thrift_type(t):
    table = {
        'float'    : 'double',
        'double'   : 'double',
        'char'     : 'byte',
        'int8_t'   : 'byte',
        'uint8_t'  : 'byte',
        'uint8_t_mavlink_version' : 'byte',
        'int16_t'  : 'i16',
        'uint16_t' : 'i16',
        'int32_t'  : 'i32',
        'uint32_t' : 'i32',
        'int64_t'  : 'i64',
        'uint64_t' : 'i64'}
    return table[t]

def thrift_array(t):
    return ('list<' + thrift_type(t) + '>')

##########################

def enumname(s):
    if (string.find(s,"MAV_") >= 0):
        return string.capwords(string.replace(s,"MAV_","",1).encode('ascii'))
    elif (string.find(s,"MAVLINK_") >= 0):
        return string.capwords(string.replace(s,"MAVLINK_","",1).encode('ascii'))

def enumentryname(s, parent):
    parent = parent.encode('ascii')
    # handle exceptions first, because mavlink is awfully inconsistent:
    if parent == "MAV_MISSION_RESULT":
        return string.replace(s,"MAV_MISSION_","",1).encode('ascii')
    elif parent == "MAV_COMPONENT":
        return string.replace(s,"MAV_COMP_ID_","",1).encode('ascii')
    elif parent == "MAVLINK_DATA_STREAM_TYPE":
        return string.replace(s,"MAVLINK_DATA_STREAM_","",1).encode('ascii')
    elif string.find(s,parent) >= 0:
        # base case
        return string.replace(s,parent+"_","",1).encode('ascii')
    else:
        # give up
        return s.encode('ascii')

def generate_mavlink_thrift(xml):
    print ("generate_mavlink_thrift for xml " + xml.basename)
    ns = "mavlink." + xml.basename
    doc = TDoc(xml.basename)
    doc.namespace("cpp",ns)
    for e in xml.enum:
        te = TEnum(enumname(e.name))
        for ee in e.entry:
            entryname = enumentryname(ee.name, e.name)
            if entryname != "ENUM_END":
                te.entry(entryname, ee.value)
        doc.enum(te)

    print (doc.pp())
    return
    for msg in xml.message:
        print ("  message " + msg.name)
        for field in msg.fields:
            if field.array_length == 0:
                print ("    field " + field.name + " :: " + field.type)
            else:
                print ("    field " + field.name + " :: " + field.type
                        + "[" + str(field.array_length) + "]")

def generate_message_debug(xml):
    print ("debug mavlink for " + xml.basename)

    for enum in xml.enum:
        print ("  enum " + enum.name)
        for entry in enum.entry:
            print ("    entry " + entry.name + " = " + str(entry.value))
    return
    for msg in xml.message:
        print ("  message " + msg.name)
        for field in msg.fields:
            if field.array_length == 0:
                print ("    field " + field.name + " :: " + field.type)
            else:
                print ("    field " + field.name + " :: " + field.type
                        + "[" + str(field.array_length) + "]")
def generate_messages(basename, xml_list):
    for xml in xml_list:
        mavgen_process_xml.process_xml(basename, xml);
        mavparse.mkdir_p(basename)
        generate_mavlink_thrift(xml)
