clc
clear all
close all

% Path to utility functions for importing and plotting simulation data
addpath('D:\c.sunday\Documents\Chrono\04_project_source\isae-sims\matlab\_util');

% Specify the data directory containing the simulation files to process
dpath = 'D:\c.sunday\Documents\Chrono\00_data\cyl\set\220608_101705';

% Import standard simulation files into matlab as data tables
energy = import_energy(dpath);
params = import_params(dpath);
packing = import_packing(dpath);
stats = import_stats(dpath);

% Import the final state data for all of the particles in the system
fstate = import_state_bin(strcat(dpath,'\state_final.bin'));

% Generate system plots
plot_energy(energy);
plot_stats(stats);
plot_grain_distribution(fstate);