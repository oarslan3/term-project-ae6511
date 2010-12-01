%function snsummary( filename )
%     Causes SNOPT to write summarized information about its progress in
%     solving the current problem to the file named in "filename."
%
%     "snsummary off" causes SNOPT to stop writing to filename,
%     and close the file.
%
%     Note that until the file has been closed, it is unlikely to contain
%     all of the output.
%     WARNING:  Do not use snset() or snseti() to set the summary file.
function snsummary( filename )

%opensummary  = snoptcmex( 0, 'SetSummaryFile'   );
%closesummary = snoptcmex( 0, 'CloseSummaryFile' );

opensummary  = 11;
closesummary = 13;

if strcmp( filename, 'off' )
  snoptcmex( closesummary );
else
  snoptcmex( opensummary, filename );
end
