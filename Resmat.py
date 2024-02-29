# MONTANDO ELEMENTOS DO AÇO 1020
# rho = int(7810)
# E = int(205e9) esses eram os parametros do Ross da petro.
# G_s = int(80e9)
# resistencia_tracao = Tr     
# resistencia_cisalhamento = Cis
# dureza_brinell = Dbr
# densidade = Dens
# modulo_elasticidade = ME 
# Q= momento estático da área A’ em relação à LN (linha neutra)
# τ = tensão de cisalhamento no elemento
# V = força de cisalhamento interna resultante
# I = momento de inércia da área da seção transversal inteira
# t = largura da área da seção transversal do elemento
class Material:
    def __init__(self, tensao_type=None, rho=None, E=None, G_s=None, V=None, Q=None, I=None, t=None, Tr=None, Cis=None,
                 Dbr=None, Dens=None, ME=None):
        if tensao_type is None:
            self.tensao_type = 'Tensão de cisalhamento'
        else:
            self.tensao_type = tensao_type

        self.V = V
        self.Q = Q
        self.I = I
        self.t = t
        #self.rho = rho
        #self.E = E
        #self.G_s = G_s
        self.Tr = Tr if Tr is not None else 420e6
        self.Cis = Cis if Cis is not None else 370e6
        self.Dbr = Dbr if Dbr is not None else 121
        self.Dens = Dens if Dens is not None else 7850
        self.ME = ME if ME is not None else 200e9

    def tensão_cisalhamento(self):
        if self.tensao_type == 'Tensão de cisalhamento':
            tensão = (self.V * self.Q) / (self.I * self.t)
            return tensão

    def resmat(self):
        if self.tensão_cisalhamento() > self.Cis:
            return "reprovado"
        else:
            return "aprovado"


Parametros = Material(V=450e9, Q=50, I=200, t=10) 

print("Tensão de cisalhamento:", Parametros.tensão_cisalhamento())
print("Resultado da avaliação do material:", Parametros.resmat())