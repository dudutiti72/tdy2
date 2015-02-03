%% Changelog

% 1.Categorized functions in folders
%   - Only functions in "Computations" and "Input Handling" folders, 
%     together with tRTUf2011.m and visres.m are necessary to perform any  
%     simulation built with the GUI version

% 2.In all functions and subfunctions of these two folders help comments
%   have been added to describe the use, as well as the inputs and outputs.
%   These comments come up with the help command in the command window and
%   can also be folded to take up less space

% 3.Corrected misspellings and grammar errors in existing comments

% 4.Structural changes in spacing and alignment to improve readability of 
%   the code

% 5.Absolutely NO changes in the code itself in this version.

% 6.Added functions comp_X and UIC_BW from previous version to avoid errors
%   when these functions are called

% 7.Wrote info.m files to explain the most important struct arrays of the
%   program (train, loco etc)

% 8.Put compFlongxm inside visres.m so that only one mfile is needed for
%   saving the results

% 9.Look for the abbreviations below for further points where comments or
%   suggestions have been added


%% Nomenclature

% [b]   : Bug

% [SBB] : Comment added after the distribution of the latest version

% [?]   : Not clear how to describe a variable or
%         Not sure how to interpret existing comment or 
%         Not sure if a comment added is totally correct

% [n]   : Not implemented in GUI

% [s!]  : Suggestion to clear up code or to correct a minor mistake

% [f]   : New feature

%% General suggestions and comments

% change findstr with strfind following Matlab's suggestion for faster
% computations

% Maybe make fsl global variable instead of input in all the functions that
% need it. The same for sep e.t.c

% Of course, GUI.inf must be edited to point the correct test file before 
% running tRTUf2011.m and 'Input Handling','Computations' folders must be
% added to path
