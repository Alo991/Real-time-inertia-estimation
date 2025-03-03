Real-time inertia estimation via ARMAX model representation and synchrophasor measurements


Authors: 
Alexander Sanchez-Ocampo, Mario R. A. Paternina, José Manuel Ramos-Guerrero,
Gabriel E. Mejia-Ruiz, Juan M. Ramirez, Lucas Lugnani, Felix Munguia, 
Alejandro Zamora-Mendez, and Juan R. Rodriguez

This paper introduces the real-time implementation 
with the actual hardware architecture environment (HAE)
of an online estimation method that tracks the equivalent
time-varying inertia in power systems. The proposed method
enables automated and accurate inertia estimation, exploiting
the ARMAX model representation and the Teager-Kaiser energy
operator (TKEO) disturbance time detector. The effectiveness
and high accuracy of the proposed framework are successfully
validated in laboratory conditions with actual synchronised
measurements from Phasor Measurement Units (PMUs) over a
real-time emulated New England 39-bus system. The estimate
is achieved with a relative error ranging from 0.1% to 7%, even
under noisy conditions and atypical measurement values. The
literature reviewed does not report any estimation method that
is more accurate than the one proposed in this work.


Requierements:

1. Matlab: 2021a
2. PMU information: IP, Port, IDcode
3. Packages: Instrument Control Toolbox

Instructions

1. Unpack the download archive.
2. Open the file - Main_H_Estimation.m
3. Update the PMU information.
4. In Matlab, execute the file called Main_H_Estimation.m to enable PMU data reading.
5. To finish the reading process, it is necessary to press ctrl+c


