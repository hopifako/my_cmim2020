%% Exercise 4.10
%Solution of Exercise 4.10 from S. Lingeand H. P. Langtangen book
%Function returns the potential and kinetic enegery of an oscillating
%system 
function [E_pot , E_kin] = osc_energy(u, v, omega)
    E_pot = 0.5 * (omega * u).^2;
    E_kin = 0.5 * v.^2;
end
