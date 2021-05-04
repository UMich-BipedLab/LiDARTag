function inner_product = computeFunctionInnerProduct(point_cloud1, point_cloud2, geo_ell)
    if ~exist('geo_ell', 'var')
        geo_ell = 0.05; % 0.05
    end
    feature_ell = 10;
    geo_sig = 1e5;

    num_piont1 = size(point_cloud1, 2);
    num_piont2 = size(point_cloud2, 2);
    inner_prod_A = zeros(num_piont1, num_piont2);
    

    for i = 1:num_piont1
        feature1 = point_cloud1(4, i);
        p1 = point_cloud1(1:3, i);
        
        for j = 1:num_piont2
            feature2 = point_cloud2(4, j);
            p2 = point_cloud2(1:3, j);
            
            feature_kernel = exp( -(norm(feature1 - feature2)) / (2*feature_ell^2) );
            geometry_kernel = geo_sig * exp( -(norm(p1 - p2)) / (2*geo_ell^2) );
            
            
            inner_prod_A(i, j) = feature_kernel * geometry_kernel;
        end
    end
    inner_product = sum(sum(inner_prod_A))/(num_piont1 * num_piont2);
end