#! /usr/bin/env ruby

require 'orocos'
require 'orocos/async'
require 'rock/bundle'
require 'vizkit'
require 'optparse'

options = {}
options[:logfile] = nil

op = OptionParser.new do |opt|
    opt.banner = <<-EOD
    visualize_robot_with_log.rb [options]  </path/to/model/file>
    EOD

    opt.on '--log=LOGFILE', String, 'path to the log file' do |log|
        options[:logfile] = log
    end

    opt.on '--help', 'this help message' do
        puts opt
    end
end

args = op.parse(ARGV)
model_file = args.shift

if !model_file
    puts "missing model file argument"
    puts op
    exit 1
end

if options[:logfile].nil?
    puts "missing path to log file"
    puts op
    exit 1
end

# load log files and add the loaded tasks to the Orocos name service
log_replay = Orocos::Log::Replay.open(options[:logfile]) unless options[:logfile].nil?

# If log replay track only needed ports
joint_dispatcher = log_replay.task('joint_dispatcher') unless options[:logfile].nil?

Bundles.initialize

# Visualization fo the robot
robotVis = Vizkit.default_loader.RobotVisualization
robotVis.modelFile = model_file.dup
robotVis.setPluginName("Robot")
Vizkit.vizkit3d_widget.setPluginDataFrame("world_osg", robotVis )

#Joints positions
joint_dispatcher.port('all_joint_states').on_data do |joints,_|

    robotVis.updateData(joints)
    puts "joints #{joints.names}"
end

Vizkit.control log_replay unless options[:logfile].nil?
Vizkit.exec

