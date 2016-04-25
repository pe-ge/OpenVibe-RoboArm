function box_out = matlab_Uninitialize(box_in)
    fclose(box_in.fid);
    box_out = box_in;
end