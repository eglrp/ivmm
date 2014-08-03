#!/usr/bin/env python
#-*- coding:utf-8 -*-

def classFactory(iface):
    import plugin
    return plugin.MainPlugin( iface )
