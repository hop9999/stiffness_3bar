function r = ML_fk_tens(r0, ro, k)

    function P = cost(r)
       P = ML_potens_energy_numerical(r, r0, ro, k);
    end
options = optimoptions('fminunc', 'Display', 'off');

[r, ~] = fminunc(@cost, r0(:, 4:6), options);
r = reshape(r,3,3);
r = [r0(:, 1:3), r];
end