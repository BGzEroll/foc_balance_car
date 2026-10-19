function configure_balance_cache
% Isolated optional cache tag permits independent offline validation processes.
here=fileparts(fileparts(mfilename('fullpath')));tag=getenv('BALANCE_SIM_CACHE_TAG');
assert(isempty(tag)||~isempty(regexp(tag,'^[A-Za-z0-9_-]+$','once')),'Invalid cache tag');
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'.cache','simulink',tag), ...
 'CodeGenFolder',fullfile(here,'.cache','codegen',tag),'createDir',true);
end
