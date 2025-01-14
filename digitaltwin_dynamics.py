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



    def __init__(self, spring_type, spring_k, spring_F, spring_non_lin_coef, tire_Fz, tire_Sa, tire_Ls):
        # Modelo de mola
        self.spring_type = spring_type # Hooke, 
        self.spring_k = spring_k # rigidez da mola [N/m]
        self.spring_F = spring_F # força que a mola recebe [N]
        self.spring_non_lin_coef = spring_non_lin_coef # coeficiente de ganho não-linear
        # Modelo de pneu
        self.tire_Fz = tire_Fz  # carga vertical no pneu [N]
        self.tire_Sa = tire_Sa  # slip angle do pneu [rad]
        self.tire_Ls = tire_Ls  # longitudinal slip do pneu [Admensional]
        self.tire_type = 'Default'
        

    def Spring(self):

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



    def Tire(self, params):
        
        E, Cy, Cx, Cz, c1, c2 = params
        Cs = c1 * np.sin(2 * np.arctan(self.tire_Fz / c2))
        D = 1.5 * self.tire_Fz
        Bz = Cs / (Cz * D)
        Bx = Cs / (Cx * D)
        By = Cs / (Cy * D)
        tire_lateral_force = D * np.sin(Cy * np.arctan(By * self.tire_Sa - E * (By * self.tire_Sa - np.arctan(By * self.tire_Sa))))
        tire_auto_align_moment = D * np.sin(Cz * np.arctan(Bz * self.tire_Sa - E * (Bz * self.tire_Sa - np.arctan(Bz * self.tire_Sa))))

        return tire_lateral_force, (12 + (tire_auto_align_moment/58))



    def Transmission(self):
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
        
        return Tra

    def Examples_Tire(self, params):

        def Tire(self, params):
        
            E, Cy, Cx, Cz, c1, c2 = params
            Cs = c1 * np.sin(2 * np.arctan(self.tire_Fz / c2))
            D = 1.5 * self.tire_Fz
            Bz = Cs / (Cz * D)
            Bx = Cs / (Cx * D)
            By = Cs / (Cy * D)
            tire_lateral_force = D * np.sin(Cy * np.arctan(By * self.tire_Sa - E * (By * self.tire_Sa - np.arctan(By * self.tire_Sa))))
            tire_auto_align_moment = D * np.sin(Cz * np.arctan(Bz * self.tire_Sa - E * (Bz * self.tire_Sa - np.arctan(Bz * self.tire_Sa))))

            return tire_lateral_force, (12 + (tire_auto_align_moment/58))
    
        # Dados experimentais
        angles = np.array([-9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0])
        tire_lateral_forces_1 = np.array([-2300, -2200, -2060, -1880, -1680, -1450, -1190, -850, -430, 60, 520, 890, 1170, 1390, 1580, 1730, 1890, 2000, 2090])
        tire_auto_align_moment_1 = np.array([-28.84, -28.68, -27.21, -26.41, -27.70, -24.21, -24.15, -15.88, -4.91, 14.72, 33.80, 43.79, 46.93, 49.09, 50.90, 50.10, 50.81, 48.12, 48.83])
        
        # Instanciando a classe Dynamics
        dynamics_instance = Dynamics(spring_type="Hooke", spring_k=1000, spring_F=500, spring_non_lin_coef=0.1, tire_Fz=1500, tire_Sa=angles, tire_Ls=0.1)
        
        # Função de erro total
        def total_error(params):
            predicted_tire_lateral_forces, predicted_tire_auto_align_moment = dynamics_instance.Tire(params)
            sq_errors_lateral_force = (predicted_tire_lateral_forces - tire_lateral_forces_1) ** 2
            sq_errors_auto_align_moment = (predicted_tire_auto_align_moment - tire_auto_align_moment_1) ** 2
            total_error = np.sum(sq_errors_lateral_force) + np.sum(sq_errors_auto_align_moment)
            return total_error
        
        # Restrições dos parâmetros
        param_bounds = [(-2, 1), (1, 3), (1, 3), (1, 3), (1000, 10000), (1, 5000)]
        
        # Minimizando o erro total com restrições nos parâmetros usando busca em grade
        result = opt.brute(total_error, param_bounds, finish = None)
        
        # Imprimindo os parâmetros otimizados
        print("Parâmetros otimizados:")
        print("E:", result[0])
        print("Cy:", result[1])
        print("Cx:", result[2])
        print("Cz:", result[3])
        print("c1:", result[4])
        print("c2:", result[5])
        
        # Calculando o erro total com os parâmetros otimizados
        total_error_optimized = total_error(result)
        print("Erro total com parâmetros otimizados:", total_error_optimized)
        
        # Plotagem da curva otimizada com os dados experimentais
        predicted_tire_lateral_forces, predicted_tire_auto_align_moment = dynamics_instance.Tire(result)
        plt.figure(figsize=(18, 7))  # Definindo um tamanho para a figura
        
        # Plotagem força lateral
        plt.subplot(1, 2, 1)
        plt.plot(angles, predicted_tire_lateral_forces, label='Curva Otimizada')
        plt.scatter(angles, tire_lateral_forces_1, color='red', label='Dados Experimentais')
        plt.xlabel('Ângulo de Deslizamento Lateral (graus)')
        plt.ylabel('Força Lateral do Pneu (N)')
        plt.title('Força Lateral do Pneu - Comparação da Curva Otimizada com os Dados Experimentais')
        plt.legend()
        plt.grid(True)
        
        # Plotagem torque auto-alinhante
        plt.subplot(1, 2, 2)
        plt.plot(angles, predicted_tire_auto_align_moment, label='Curva Otimizada')
        plt.scatter(angles, tire_auto_align_moment_1, color='blue', label='Dados Experimentais')
        plt.xlabel('Ângulo de Deslizamento Lateral (graus)')
        plt.ylabel('Torque auto-alinhante (N.m)')
        plt.title('Torque Auto-alinhante - Comparação da Curva Otimizada com os Dados Experimentais')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
