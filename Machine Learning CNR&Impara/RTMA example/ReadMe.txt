
17.08.18

Once machine learning has been applied at design time , a function is derived to be applicable at run time to infer safety conditions. In this case, the function applies to vehicle platooning and shows how collision prediction is provided according to system conditions monitored at run time. The function based on a set of boolean rules may be easily implemented on the field. [Mon18] shows how the set of run time functions provide forecast with statistical error very close to zero, while keeping the largest set of working conditions.

The sw related with the item form is as follows.
•	SAFECOP_D42_CNR&Impara_RTMA model.c: the run time manager function.
•	SAFECOP_D42_CNR&Impara_RTMA model with application.c: the run time manager function applied in a simulation context. Nothing prevents to adopt the run time manager function in a real device.
•	SAFECOP_D42_CNR&Impara_RTMA model with application output.txt: the output provided by the application; it gives an example about the prediction of collision before it may happen.

[Mon18] E. Ferrari. A. Fermi, M. Mongelli, M. Muselli, “Identification of Safety Regions in Vehicle Platooning via Machine Learning,” 14th IEEE Internat. Work. on Fact. Commun. Sys. WFCS 2018.
