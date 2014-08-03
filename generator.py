#!/usr/bin/python
#-*- coding:utf-8 -*-
import ConfigParser
import pyivmm
import getopt
import os,sys
import argparse
import random
from os import path
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(sys.argv[0])
    parser.add_argument('--config','-c', dest='config_file', help='special config file (default: default.ini)', type=str, default='default.ini')
    args = parser.parse_args()

    config = ConfigParser.ConfigParser()
    if not config.read(args.config_file):
        print>>sys.stderr, 'cannot open "%s", generator a default config file' % args.config_file
        with open(args.config_file,'w') as f:
            config.add_section('Network')
            config.set('Network', 'cross', '../shp/cross')
            config.set('Network', 'road', '../shp/road')
            config.add_section('LaunchParam')
            config.set('LaunchParam', 'speed_white_error_stddev', '3')
            config.set('LaunchParam', 'time_sample_mean', '60')
            config.set('LaunchParam', 'time_sample_stddev','15')
            config.set('LaunchParam', 'speed_confidence','0.6')
            config.set('LaunchParam', 'speed_similar','0.7')
            config.set('LaunchParam', 'gps_sample_stddev','50')
            config.add_section('Result')
            config.set('Result', 'output','outs')
            config.set('Result', 'group_count', '100')
            config.set('Result', 'k_shortest', '5')
            config.write(f)
        sys.exit(1)

    try:
        network = pyivmm.Network(config.get('Network','cross'), config.get('Network', 'road'))
    except:
        print>>sys.stderr, 'build network fail, check files'
        sys.exit(1)
    
    sample = pyivmm.SampleGenerator(network)

    launchParam = pyivmm.LaunchParam()
    launchParam.gps_sample_stddev = config.getfloat('LaunchParam', 'gps_sample_stddev')
    launchParam.speed_confidence = config.getfloat('LaunchParam', 'speed_confidence')
    launchParam.speed_similar = config.getfloat('LaunchParam', 'speed_similar')
    launchParam.speed_white_error_stddev = config.getfloat('LaunchParam', 'speed_white_error_stddev')
    launchParam.time_sample_mean = config.getfloat('LaunchParam', 'time_sample_mean')
    launchParam.time_sample_stddev = config.getfloat('LaunchParam', 'time_sample_stddev')


    outdir = config.get('Result', 'output')
    sample_out_dir = os.path.join(outdir, 'sample')
    origin_out_dir = os.path.join(outdir, 'origin')
    path_out_dir = os.path.join(outdir, 'path')

    def random_two_cross():
        a = random.randint(0, network.cross_bound() - 1)
        b = random.randint(0, network.cross_bound() - 1)
        while a == b:
            a = random.randint(0, network.cross_bound() - 1)
            b = random.randint(0, network.cross_bound() - 1)
        return (a, b)

    group_count = config.getint('Result', 'group_count')
    k_shortest = config.getint('Result', 'k_shortest')
    if not path.isdir(sample_out_dir):
        os.makedirs(sample_out_dir)
    if not path.isdir(origin_out_dir):
        os.makedirs(origin_out_dir)
    if not path.isdir(path_out_dir):
        os.makedirs(path_out_dir)
    
    dist = []
    for i in xrange(group_count):
        begin, end = random_two_cross()
        results = sample.launch(begin, end, k_shortest, launchParam)
        while len(results) == 0:
            begin, end = random_two_cross()
            results = sample.launch(begin, end, k_shortest, launchParam)

        for j, sample_result in enumerate(results):
            sample_file_name = path.join(sample_out_dir, '%d_%d.log'%(i,j))
            origin_file_name = path.join(origin_out_dir, '%d_%d.log'%(i,j))
            path_file_name = path.join(path_out_dir, '%d_%d.log'%(i,j))
            #dist.extend(network.project(g).gis_dist(g) for g in sample_result.sample)
            for g, org in zip(sample_result.sample, sample_result.origin):
                pp = org.belong.candidate_of(g)
                d = pp.gis_dist(g)
                dist.append(d)
            #with open(sample_file_name, 'w') as sample_file:
            #    sample_file.write('#lng,lat,timestamp\n')
            #    sample_file.writelines(('%g, %g, %ld\n'%(g.x, g.y, g.timestamp) for g in sample_result.sample))
            #    dist.extend(network.project(g).gis_dist(g) for g in sample_result.sample)
            #with open(origin_file_name, 'w') as origin_file:
            #    origin_file.write('#lng,lat,road id,where(normaled),timestamp\n')
            #    origin_file.writelines(('%g,%g,%d,%g,%ld,\n'%(g.x, g.y, g.belong.id, g.where, g.timestamp) for g in sample_result.origin))
            #with open(path_file_name,'w') as path_file:
            #    path_file.write('#road id, cross id begin, cross id end\n')
            #    points = filter(lambda x:x.cid != -1, sample_result.path.points)
            #    path_file.writelines('%d,%d,%d\n'%(p[0].belong.id,p[0].cid,p[1].cid) 
            #        for p in zip(points[0:-1], points[1:]) if p[0].belong == p[1].belong)
    
    with open(path.join(outdir,"stat"), 'w') as fstat:
        fstat.write('%g\n'%np.mean(dist))
        fstat.write('%g\n'%np.std(dist))
    with open(path.join(outdir, 'dist'), 'w') as fdist:
        fdist.writelines('%g\n'% d for d in dist)
