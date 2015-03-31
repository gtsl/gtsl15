delx = 5;
dely = 5;
% table = generate_table(delx, dely);
% full_table = imresize(table, delx);
full_table(full_table > 90) = 90;
full_table(full_table < 0) = 0;
surfc(full_table, 'EdgeColor', 'none')
% full_table = interp_table(delx, dely);