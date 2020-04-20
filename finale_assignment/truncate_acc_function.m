function acc_fun = truncate_acc_function(acc_function,t,q,qip)
    acc_fun = acc_function(t,q,qip);
    acc_fun = acc_fun(1:12);
end