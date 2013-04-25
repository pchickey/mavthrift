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

# escape reserved words
def thrift_name(field, m):
    f = field.encode('ascii')
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

##########################

def enumname(s):
    r = ""
    if (string.find(s,"MAV_") >= 0):
        r = string.replace(s,"MAV_","",1)
    elif (string.find(s,"MAVLINK_") >= 0):
        r = string.replace(s,"MAVLINK_","",1)
    else:
        r = s
    return string.capwords(r.encode('ascii'))

def enumentryname(s, parent):
    parent = parent.encode('ascii')
    # handle exceptions first, because mavlink is awfully inconsistent:
    if parent == "MAV_MISSION_RESULT":
        return string.replace(s,"MAV_MISSION_","",1).encode('ascii')
    elif parent == "MAV_COMPONENT":
        return string.replace(s,"MAV_COMP_ID_","",1).encode('ascii')
    elif parent == "MAVLINK_DATA_STREAM_TYPE":
        return string.replace(s,"MAVLINK_DATA_STREAM_","",1).encode('ascii')
    elif parent == "LIMITS_STATE":
        return string.replace(s,"LIMITS_","",1).encode('ascii')
    elif parent == "LIMIT_MODULE":
        return string.replace(s,"LIMIT_","",1).encode('ascii')
    elif string.find(s,parent) >= 0:
        # base case
        return string.replace(s,parent+"_","",1).encode('ascii')
    else:
        # give up
        return s.encode('ascii')

def generate_mavlink_thrift(xml, parent):
    print ("generate_mavlink_thrift for xml " + xml.basename)
    ns = "mavlink.thrift"
    doc = TDoc(xml.basename)
    doc.namespace("cpp",ns)

    typeenum = TEnum(leadingcap(xml.basename) + 'MessageTypes')
    if parent:
        for msg in parent.message:
            typeenum.entry(msg.name)
    for msg in xml.message:
        typeenum.entry(msg.name)

    doc.enum(typeenum)

    for e in xml.enum:
        te = TEnum(enumname(e.name))
        for ee in e.entry:
            entryname = enumentryname(ee.name, e.name)
            if entryname != "ENUM_END":
                te.entry(entryname, ee.value)
        doc.enum(te)


    for msg in xml.message:
        ts = TStruct(msg.name_module)
        for field in msg.fields:
            tname = thrift_name(field.name, msg)
            ttype = thrift_type(field.type)
            if field.array_length == 0:
                ts.atom_field(tname, ttype)
            else:
                ts.list_field(tname, ttype)
        doc.struct(ts)
    generate_message_post_service(xml,doc)
    generate_message_fetch_service(xml,doc,typeenum)
    return doc.pp()

def leadingcap(s):
    return s[:1].upper() + s[1:]

def generate_message_post_service(xml,doc):
    if (xml.basename == "common"):
        ex = TException("InvalidMavlinkMessage")
        ex.atom_field("error","string")
        doc.exception(ex)
    else:
        ex = TException("common.InvalidMavlinkMessage")
        doc.include('common.thrift')

    service = TService(leadingcap(xml.basename) + "MessagePost")
    if (xml.basename != "common"):
        service.extends("common.CommonMessagePost")

    for s in doc.structs:
        m = TVoidMethod("post" + leadingcap(s.typename))
        m.arg(s,"msg")
        m.throws(ex,"err")
        service.method(m)
    doc.service(service)

def generate_message_fetch_service(xml,doc,typeenum):
    service = TService(leadingcap(xml.basename) + "MessageFetch")
    if (xml.basename != "common"):
        service.extends("common.CommonMessageFetch")

    service.method(TMethod("availableMessages", TMap(typeenum,TTyped('i32'))))

    for s in doc.structs:
        m = TMethod("fetch" + leadingcap(s.typename),TList(s))
        service.method(m)

    doc.service(service)

def generate_mavlink_debug(xml):
    print ("debug mavlink for " + xml.basename)
    if False:
        for enum in xml.enum:
            print ("  enum " + enum.name)
            for entry in enum.entry:
                print ("    entry " + entry.name + " = " + str(entry.value))
    for msg in xml.message:
        print ("  message " + msg.name_module)
        for field in msg.fields:
            if field.array_length == 0:
                print ("    field " + field.name + " :: " + field.type)
            else:
                print ("    field " + field.name + " :: " + field.type
                        + "[" + str(field.array_length) + "]")

def writethrift(t, outputdir, basename):
    fname = os.path.join(outputdir, basename + ".thrift")
    f = open(fname, 'w')
    f.write(t)
    f.close()

def generate(xml_list, outputdirectory):
    mavparse.mkdir_p(outputdirectory)

    # I've really punted on doing dependencies right -
    # it would take some significant work.
    # We'll go back to it later.

    common = None
    for xml in xml_list:
        mavgen_process_xml.process_xml(xml)
        if xml.basename == "common":
            common = xml

    for xml in xml_list:
        if xml.basename == "common":
            r = generate_mavlink_thrift(xml, None)
        else:
            r = generate_mavlink_thrift(xml, common)
        writethrift(r,outputdirectory,xml.basename)

