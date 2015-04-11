function write_text(control_matrix, del)

% Write 2d control matrix of size (3000, 500) to c header file
size(control_matrix)
% Get parent directory path
path = mfilename('fullpath');
cnt = 0;
ndx = length(path);
while cnt < 2
    if path(ndx) == '/'
        cnt = cnt + 1;
    end
    path(ndx) = [];
    ndx = ndx - 1;
end
path(ndx + 1) = '/';

% Create file
path = [path 'control_matrix.h'];
fh = fopen(path, 'wt');

% Write to file 
fprintf(fh, 'uint8_t control_matrix_delta = %d;\n', del);
fprintf(fh, 'uint8_t control_matrix[%d][%d] = {\n', 2500/del, 500/del);
for i = 1:(2500/del - 1)
    fprintf(fh, '{');
    fprintf(fh, '%.0f,', control_matrix(i,1:end-1));
    fprintf(fh, '%.0f},\n', control_matrix(i,end));
end
% Handle last line special case
fprintf(fh, '{');
fprintf(fh, '%.0f,', control_matrix(i,1:end-1));
fprintf(fh, '%.0f}', control_matrix(i,end));
fprintf(fh, '};');
fclose(fh);
end