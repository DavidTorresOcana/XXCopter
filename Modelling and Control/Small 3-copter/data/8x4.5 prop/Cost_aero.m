function cost = Cost_aero(C_TModel,C_QModel,sigma,theta_0,theta_1,a,c_d_0)

C_T_theo = sigma*a/4*(theta_0*2/3+theta_1/2+C_TModel(:,1));
C_Q_theo = sigma*a/4*(theta_0*2/3+theta_1/2+C_QModel(:,1)).*C_QModel(:,1) + sigma*c_d_0/8;


cost=1*sum((C_T_theo -C_TModel(:,2)).^2) +1* sum((C_Q_theo -C_QModel(:,2)).^2);

end