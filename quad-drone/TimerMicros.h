class TimerMicros{
private:
    unsigned long intervalo;
    unsigned long ultimaExecucao;

public:
    TimerMicros(unsigned long intervalo_ms) {
        intervalo = intervalo_ms;
        ultimaExecucao = micros();
    }

    bool pronto() {
        unsigned long agora = micros();
        if (agora - ultimaExecucao >= intervalo) {
            ultimaExecucao = agora;
            return true;
        }
        return false;
    }

    void reiniciar() {
        ultimaExecucao = micros();
    }

    void setIntervalo(unsigned long novo_intervalo) {
        intervalo = novo_intervalo;
    }

    unsigned long getIntervalo() const {
        return intervalo;
    }
};
