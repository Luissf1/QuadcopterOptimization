
% Configuración inicial
figure;
hold on;
grid on;
xlabel('Iteración');
ylabel('Fitness (J)');
title('Convergencia Combinada de las 5 Pruebas');
colors = lines(5); % Asigna colores distintos (azul, rojo, amarillo, etc.)

% Lista de nombres de archivos de las imágenes
image_files = {
    'Convergencia_Test_1.png', 
    'Convergencia_Test_2.png', 
    'Convergencia_Test_3.png', 
    'Convergencia_Test_4.png', 
    'Convergencia_Test_5.png'
};

% Procesar cada imagen
for test_num = 1:5
    % Cargar imagen
    img = imread(image_files{test_num});
    
    % Convertir a escala de grises y detectar la curva (ajusta el umbral)
    img_gray = rgb2gray(img);
    curve_mask = img_gray < 50; % Umbral para detectar líneas oscuras
    
    % Extraer coordenadas de los píxeles de la curva
    [y, x] = find(curve_mask);
    
    % Normalizar coordenadas (ajusta según tus ejes originales)
    x_norm = (x - min(x)) / (max(x) - min(x)) * 100; % Eje X: 0-100 iteraciones
    y_norm = (max(y) - y) / (max(y) - min(y)) * 0.3; % Eje Y: Fitness escalado a [0, 0.3]
    
    % Graficar (con transparencia para superposición)
    scatter(x_norm, y_norm, 10, colors(test_num,:), 'filled', ...
           'DisplayName', sprintf('Prueba %d', test_num), ...
           'MarkerFaceAlpha', 0.5); % Transparencia del 50%
end

% Añadir leyenda y ajustar ejes
legend('show', 'Location', 'northeast');
xlim([0 100]); % Límites del eje X (iteraciones)
ylim([0 0.3]); % Límites del eje Y (fitness)
hold off;

% Guardar la figura combinada
saveas(gcf, 'Convergencia_Combinada.png');
disp('Figura combinada guardada como: Convergencia_Combinada.png');