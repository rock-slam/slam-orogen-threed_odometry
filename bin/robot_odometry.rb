#!/usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
require 'utilrb'

include Orocos

op = OptionParser.new do |opt|
    opt.banner = <<-EOD
    usage: robot_odometry
    EOD

    opt.on '--help', 'this help message' do
        puts opt
       exit 0
    end
end

Orocos::CORBA::max_message_size = 100000000000
Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'threed_odometry::Task' => 'robot_odometry', :gdb => false do

    # Get the task names from odometry
    robot_odometry = Orocos.name_service.get 'robot_odometry'

    # Set configuration files for odometry
    Orocos.conf.apply(robot_odometry, ['default', 'bessel50'], :override => true)
    robot_odometry.urdf_file = Bundles.find_file('data/odometry', 'exoter_odometry_model.urdf')

    ###################
    ## LOG THE PORTS ##
    ###################
    Bundles.log_all

    # Configure tasks from odometry
    robot_odometry.configure

    # Start tasks from odometry
    robot_odometry.start

    Vizkit.exec
end
