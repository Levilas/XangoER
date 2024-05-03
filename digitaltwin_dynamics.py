class Dynamics:
    """A Vehicle Dynamics object.

    This class will create a Vehicle Dynamics digital twin with the springs, suspension bars, damper cylinder, brakes, tires, and transmission elements provided.

    Parameters AINDA A ALTERAR, ISSO É UM EXEMPLO
    ----------
    shaft_elements : list
        List with the shaft elements.
    disk_elements : list
        List with the disk elements.
    bearing_elements : list
        List with the bearing elements.
    automeshing : boolean
        Set it True to use the automeshing method. Default is False.
        If automeshing is True, the previous shaft_elements parameter is now defined by:
            shaft_elements : list
                List with the length, inner and outter diameter and material of each element, as follows:
                    [[length, inner, outter, material],[length, inner, outter, material],...]
        For the other parameters please check the respective bearing and disk classes for more information.
    **kwargs : dict, optional
        If automeshing is True, these parameters needs to be informed.
        The possible arguments are:
            alpha : float
                Proportional damping coefficient, associated to the element Mass matrix
            beta : float
                Proportional damping coefficient, associated to the element Stiffness matrix

    Returns
    -------
    A rotor object.

    Attributes
    ----------
    MM : array
        Global mass matrix.
    KK : array
        Global stiffness matrix.
    CCgyros: array
        Global gyroscopic matrix.
    CCtotal: array
        Global damping matrix

    Examples
    --------
    >>> import lmest_rotor as lm
    >>> rotor = lm.rotor_example()
    >>> rotor.MM
    array(30x30)
    """



    def __init__(self, spring_type, spring_k, spring_F):
        self.spring_type = spring_type # Hooke, 
        self.spring_k = spring_k # rigidez da mola [N/m]
        self.spring_F = spring_F # força que a mola recebe [N]
        self.spring_non_lin_coef = spring_non_lin_coef # coeficiente de ganho não-linear
        

    def Spring(self):
        """Description.

        Detailed description.

        Returns
        -------
        Bat : variable type
            Description.

        Examples
        --------
        >>> example
        """
        if self.spring_type == 'Hooke':
            spring_x = self.spring_F/self.spring_k
        if self.spring_type == 'Softening'
            spring_x = self.spring_F/(self.spring_non_lin_coef*(self.spring_k)**2)

        return spring_x



    def Suspension(self):
        """Description.

        Detailed description.

        Returns
        -------
        Bat : variable type
            Description.

        Examples
        --------
        >>> example
        """
        
        return Sus



    def Damper(self):
        """Description.

        Detailed description.

        Returns
        -------
        Bat : variable type
            Description.

        Examples
        --------
        >>> example
        """
        
        return Dam

    

    def Brake(self):
        """Description.

        Detailed description.

        Returns
        -------
        Bat : variable type
            Description.

        Examples
        --------
        >>> example
        """
        
        return Bra



    def Tire(self):
        """Description.

        Detailed description.

        Returns
        -------
        Bat : variable type
            Description.

        Examples
        --------
        >>> example
        """
        
        return Tir



    def CalculateOutputs(self, cgx, cgy, massa, etex, cfat, rpneu, acpi, redp, red1, cp):
        peso = massa * 9.81
        rnet = (peso * (cgx * 0.001)) / (etex * 0.001)
        rned = massa * 9.81 - rnet
        ftr = (rnet * cfat) / (1 - ((cgy * 0.001) * cfat) / (etex * 0.001))
        tdcl = (ftr * cgy * 0.001) / (etex * 0.001)
        cnet = rnet + tdcl
        ptet = cnet * rpneu * 0.001 * cfat
        cpneu = cnet / 2
        tpneu = ftr * rpneu * 0.001
        redf = redp * red1
        tpwt = ptet / (red1 * redp * cp)
        acpr = (ftr / massa) / 9.81
        acfi = acpi * 9.81
        acfr = acpr * 9.81
        fti = massa * acfi
        tpi = fti * rpneu * 0.001
        tpwti = tpi / (red1 * redp * cp)
        tci = (fti * cgy) / etex
        tcr = (ftr * cgy) / etex
        cteti = rnet + tci
        return peso, rnet, rned, ftr, tdcl, cnet, ptet, cpneu, tpneu, redf, tpwt, acpr, acfi, acfr, fti, tpi, tpwti, tci, tcr, cteti

    def CurveTorquePower(self, data_matrix):
        rpm_values = []
        torque_values = []
        power_values = []
        for data in data_matrix:
            rpm = data["rpm"]
            ptc = data["ptc"]
            trq = data["trq"]
            rpm_values.append(rpm)
            torque_values.append(trq)
            power_values.append(ptc)
        return rpm_values, torque_values, power_values

    def Transmission(self, cgx, cgy, massa, etex, cfat, rpneu, acpi, redp, red1, cp):
        peso, rnet, rned, ftr, tdcl, cnet, ptet, cpneu, tpneu, redf, tpwt, acpr, acfi, acfr, fti, tpi, tpwti, tci, tcr, cteti = self.CalculateOutputs(cgx, cgy, massa, etex, cfat, rpneu, acpi, redp, red1, cp)

        # Valores experimentais para a curva de torque e potência
        matriz_dados = [
            {"rpm": 0, "ptc": 0.0, "trq": 200.0},
            {"rpm": 375, "ptc": 1.5, "trq": 198.5},
            {"rpm": 750, "ptc": 4.0, "trq": 197.0},
            {"rpm": 1125, "ptc": 7.0, "trq": 195.5},
            {"rpm": 1500, "ptc": 10.5, "trq": 194.0},
            {"rpm": 1875, "ptc": 14.0, "trq": 192.5},
            {"rpm": 2250, "ptc": 17.5, "trq": 191.0},
            {"rpm": 2625, "ptc": 21.0, "trq": 189.5},
            {"rpm": 3000, "ptc": 24.5, "trq": 188.0},
            {"rpm": 3375, "ptc": 28.0, "trq": 186.5},
            {"rpm": 3750, "ptc": 31.5, "trq": 185.0},
            {"rpm": 4125, "ptc": 35.0, "trq": 183.5},
            {"rpm": 4500, "ptc": 38.5, "trq": 182.0},
            {"rpm": 4875, "ptc": 42.0, "trq": 180.5},
            {"rpm": 5250, "ptc": 45.5, "trq": 179.0},
            {"rpm": 5625, "ptc": 49.0, "trq": 177.5},
            {"rpm": 6000, "ptc": 52.5, "trq": 176.0},
            {"rpm": 6375, "ptc": 56.0, "trq": 174.5},
            {"rpm": 6750, "ptc": 59.5, "trq": 173.0},
            {"rpm": 7000, "ptc": 63.0, "trq": 171.5},
            {"rpm": 7375, "ptc": 66.0, "trq": 167.5},
            {"rpm": 7750, "ptc": 69.0, "trq": 163.5},
            {"rpm": 8125, "ptc": 72.0, "trq": 159.5},
            {"rpm": 8500, "ptc": 75.0, "trq": 155.5},
            {"rpm": 8875, "ptc": 77.5, "trq": 151.5},
            {"rpm": 9250, "ptc": 80.0, "trq": 147.5},
            {"rpm": 9625, "ptc": 81.5, "trq": 143.5},
            {"rpm": 10000, "ptc": 83.0, "trq": 139.5},
            {"rpm": 10375, "ptc": 84.5, "trq": 135.5},
            {"rpm": 10750, "ptc": 86.0, "trq": 131.5},
            {"rpm": 11125, "ptc": 87.5, "trq": 127.5},
            {"rpm": 11500, "ptc": 89.0, "trq": 123.5},
            {"rpm": 11875, "ptc": 90.5, "trq": 119.5},
            {"rpm": 12250, "ptc": 92.0, "trq": 115.5},
            {"rpm": 12625, "ptc": 93.5, "trq": 111.5},
            {"rpm": 13000, "ptc": 95.0, "trq": 107.5},
            {"rpm": 13375, "ptc": 96.5, "trq": 103.5},
            {"rpm": 13750, "ptc": 98.0, "trq": 99.5},
            {"rpm": 14125, "ptc": 99.5, "trq": 95.5},
            {"rpm": 14500, "ptc": 100.0, "trq": 91.5},
            {"rpm": 14875, "ptc": 100.0, "trq": 87.5},
            {"rpm": 15000, "ptc": 100.0, "trq": 85.0}
        ]

        # Print dos resultados obtidos
        print("Resultados:")
        print("Peso:", peso)
        print("Reação no eixo traseiro (N):", rnet)
        print("Reação no eixo dianteiro (N):", rned)
        print("Força de tração resultante:", ftr)
        print("Torque de deslocamento da carga:", tdcl)
        print("Força vertical sobre o eixo:", cnet)
        print("Potência total transmitida pelo eixo:", ptet)
        print("Carga vertical no pneu:", cpneu)
        print("Torque transmitido pelo pneu:", tpneu)
        print("Redução final:", redf)
        print("Torque na roda:", tpwt)
        print("Aceleração resultante:", acpr)
        print("Aceleração de frenagem:", acfi)
        print("Aceleração de frenagem na roda:", acfr)
        print("Força de tração inicial:", fti)
        print("Torque na entrada da roda:", tpi)
        print("Torque na entrada da roda com a transmissão:", tpwti)
        print("Torque transmitido pela coroa:", tci)
        print("Torque transmitido pelo conjunto coroa-pinhão:", tcr)
        print("Carga transmitida pela engrenagem ao eixo traseiro:", cteti)

        print("\nMatriz de RPM, Torque e Potência:")
        print("RPM\t\tTorque (Nm)\tPotência (%)")
        for data in matriz_dados:
            rpm = data["rpm"]
            trq = data["trq"]
            ptc = data["ptc"]
            print("{:.2f}\t\t{:.2f}\t\t{:.2f}".format(rpm, trq, ptc))

        # Plotando o gráfico
        rpm_values, torque_values, power_values = self.CurveTorquePower(matriz_dados)
        plt.figure(figsize=(10, 6))
        plt.plot(rpm_values, torque_values, label="Torque")
        plt.plot(rpm_values, power_values, label="Potência")
        plt.title("Curva de Torque e Potência")
        plt.xlabel("RPM")
        plt.ylabel("Torque / Potência")
        plt.legend()
        plt.grid(True)
        plt.show()

        return peso, rnet, rned, ftr, tdcl, cnet, ptet, cpneu, tpneu, redf, tpwt, acpr, acfi, acfr, fti, tpi, tpwti, tci, tcr, cteti, matriz_dados

        # Criando uma instância da classe e chamando o método Transmission
        dynamics = Dynamics()
        cgx = 853  # mm
        cgy = 294  # mm
        massa = 347  # kg
        etex = 1567  # mm
        cfat = 0.9  # coeficiente de atrito
        rpneu = 259  # mm
        acpi = 1.2  # g
        redp = 2.12  # redução primária
        red1 = 2.76  # redução da marcha única
        cp = 4  # relação coroa-pinhão
        dynamics.Transmission(cgx, cgy, massa, etex, cfat, rpneu, acpi, redp, red1, cp)
        
       


