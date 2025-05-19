function q_result = quaternionMultiply(q1, q2)
    w_0 = q1(1); w_1 = q1(2); w_2 = q1(3); w_3 = q1(4);
    p_0 = q2(1); p_1 = q2(2); p_2 = q2(3); p_3 = q2(4);
    
    scalarPart = w_0 * p_0 - w_1 * p_1 - w_2 * p_2 - w_3 * p_3;
    vectorPart = [w_0 * p_1 + w_1 * p_0 + w_2 * p_3 - w_3 * p_2;
                  w_0 * p_2 - w_1 * p_3 + w_2 * p_0 + w_3 * p_1;
                  w_0 * p_3 + w_1 * p_2 - w_2 * p_1 + w_3 * p_0];
    
    q_result = [scalarPart; vectorPart];
end