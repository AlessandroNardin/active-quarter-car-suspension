%% =========================================================================
%  main.m  -  Progetto "Controllo Robusto" (Traccia n.1)
%  Sospensione attiva quarter-car con due attuatori.
%
%  Esegue l'intera pipeline:
%    0) test_model      : costruzione e verifica del modello
%    1) design_lqg      : Task 1 - LQG (con e senza integratore)
%    2) design_hinf     : Task 2 - Hinf (mixsyn, hinfsyn, hinfstruct)
%    3) design_musyn    : Task 3 - mu-synthesis (DK-iteration) + mu-analysis
%    4) analysis_compare: Task 4 - confronto ; Task 5 - robustezza
%
%  Uso:  >> main
% =========================================================================
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here); cd(here);

fprintf('\n############ TASK 0: modello ############\n');
test_model;

fprintf('\n############ TASK 1: LQG ############\n');
design_lqg;

fprintf('\n############ TASK 2: H-infinity ############\n');
design_hinf;

fprintf('\n############ TASK 3: mu-synthesis ############\n');
design_musyn;

fprintf('\n############ TASK 4-5: confronto e robustezza ############\n');
analysis_compare;

fprintf('\n############ COMPLETATO. Figure in ../results/fig ############\n');
