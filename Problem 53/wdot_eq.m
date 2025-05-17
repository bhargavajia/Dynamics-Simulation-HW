function wdot = wdot_eq(t, w, I)
    I = diag(I);

    % Angular momentum 
    H = I * w;

    % Compute the derivative of w
    wdot = I \ (-cross(w, H));
end
