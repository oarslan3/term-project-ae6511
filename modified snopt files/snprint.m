% function snprint( filename )
%     Causes SNOPT to write detailed information about its progress in
%     solving the current problem to the file named in "filename."
%
%     "snprint off" causes SNOPT to stop writing to filename,
%     and close the file.
%
%     Note that until the file has been closed, it is unlikely to contain
%     all of the output.
%     WARNING:  Do not use snset() or snseti() to set the print file.

function snprint( filename )

%openprintfile  = snoptcmex( 0, 'SetPrintFile'   );
%closeprintfile = snoptcmex( 0, 'ClosePrintFile' );

openprintfile  = 10;
closeprintfile = 12;

if strcmp( filename, 'off' )
  snoptcmex( closeprintfile );
elseif strcmp( filename, 'on' )
  snoptcmex( openprintfile, 'print.out' );
else
  snoptcmex( openprintfile, filename );
end
