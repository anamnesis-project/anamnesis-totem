// Conecta ao servidor WebSocket no Raspberry Pi
const socket = io();

// Pega os elementos da página que vamos atualizar
const objectTempEl = document.getElementById('objectTemp');
const dataContainer = document.getElementById('data-container');
const errorContainer = document.getElementById('error-container');

// Evento: Conectado com sucesso ao servidor
socket.on('connect', () => {
    console.log('Conectado ao servidor do totem via WebSocket!');
    errorContainer.style.display = 'none';
    dataContainer.style.display = 'block';
});

// Evento: Recebeu uma atualização de sensor!
socket.on('sensor_update', (data) => {
    // Verifica se há um erro na mensagem
    if (data.error) {
        errorContainer.textContent = `ERRO: ${data.error}`;
        errorContainer.style.display = 'block';
        dataContainer.style.display = 'none';
        return;
    }
    
    // Esconde a mensagem de erro se os dados voltarem ao normal
    errorContainer.style.display = 'none';
    dataContainer.style.display = 'block';

    console.log('Novos dados de temperatura recebidos:', data);

    // Atualiza o texto na tela com os novos dados de temperatura
    // Usamos .toFixed(2) para formatar com 2 casas decimais
    objectTempEl.textContent = data.object.toFixed(2);
});

// Evento: Conexão perdida
socket.on('disconnect', () => {
    console.log('Desconectado do servidor do totem.');
    // Mostra uma mensagem de erro na tela
    errorContainer.textContent = 'ERRO: Conexão com o servidor perdida.';
    errorContainer.style.display = 'block';
    dataContainer.style.display = 'none';
});
