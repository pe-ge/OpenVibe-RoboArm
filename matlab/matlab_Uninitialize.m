function box_out = matlab_Uninitialize(box_in)
    fclose(box_in.f_one_dim_id);
    fclose(box_in.f_raw_id);
    box_out = box_in;
end