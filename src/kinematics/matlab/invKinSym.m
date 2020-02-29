function angles = invKinSym(M_joints)
    M_target = sym(eye(4));
    M_symbols = sym('M', [3 4]);
    M_target(1:3, 1:4) = M_symbols;
    angles = containers.Map;
    known = symvar(M_symbols);
    
    % Solve for th1, th5, th6
    disp("Solving for th1, th5, th6 ...");
    invM_1 = invTransf(M_joints(:,:,1));
    invM_6 = invTransf(M_joints(:,:,6));
    lhs = invM_1 * M_target * invM_6;
    rhs = sym(eye(4));
    for j = 2:1:5
        rhs = rhs * M_joints(:,:,j);
    end
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 3); 
    
    disp(known);
    fprintf('\n');
end