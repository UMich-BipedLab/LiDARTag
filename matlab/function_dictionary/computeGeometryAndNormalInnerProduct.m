function out_t = computeGeometryAndNormalInnerProduct(opts, lidartag_data_t, aptiltag_data_t)
    out_t.ell = opts.ell;
    out_t.geometry_pc = constructGeometryPointCloud(lidartag_data_t.pionts_ideal_frame, out_t.ell);
    out_t.inner_product = computeFunctionInnerProduct(lidartag_data_t.pionts_ideal_frame, aptiltag_data_t.image_array, out_t.ell);
    out_t.geometry_inner_product = computeFunctionInnerProduct(out_t.geometry_pc, aptiltag_data_t.geometry_img, out_t.ell);
end