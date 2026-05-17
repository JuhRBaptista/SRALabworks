const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell,
  AlignmentType, HeadingLevel, BorderStyle, WidthType, ShadingType,
  LevelFormat, PageNumber, Footer, Header, TabStopType, TabStopPosition,
  UnderlineType, PageBreak
} = require('docx');
const fs = require('fs');

// ─── Helpers ───────────────────────────────────────────────────────────────

const CONTENT_W = 9026; // A4 with 1-inch margins in DXA

function h1(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_1,
    spacing: { before: 360, after: 180 },
    children: [new TextRun({ text, bold: true, size: 32, font: 'Arial' })]
  });
}

function h2(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_2,
    spacing: { before: 240, after: 120 },
    children: [new TextRun({ text, bold: true, size: 28, font: 'Arial' })]
  });
}

function h3(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_3,
    spacing: { before: 200, after: 100 },
    children: [new TextRun({ text, bold: true, size: 26, font: 'Arial' })]
  });
}

function para(text, opts = {}) {
  return new Paragraph({
    alignment: opts.align || AlignmentType.JUSTIFIED,
    spacing: { before: 100, after: 100 },
    children: [new TextRun({ text, size: 24, font: 'Arial', ...opts })]
  });
}

function paraRuns(runs, opts = {}) {
  return new Paragraph({
    alignment: opts.align || AlignmentType.JUSTIFIED,
    spacing: { before: 100, after: 100 },
    children: runs.map(r =>
      typeof r === 'string'
        ? new TextRun({ text: r, size: 24, font: 'Arial' })
        : new TextRun({ size: 24, font: 'Arial', ...r })
    )
  });
}

function bullet(text, level = 0) {
  return new Paragraph({
    numbering: { reference: 'bullets', level },
    spacing: { before: 60, after: 60 },
    children: [new TextRun({ text, size: 24, font: 'Arial' })]
  });
}

function emptyLine() {
  return new Paragraph({ children: [new TextRun({ text: '' })] });
}

function caption(text) {
  return new Paragraph({
    alignment: AlignmentType.CENTER,
    spacing: { before: 60, after: 180 },
    children: [new TextRun({ text, size: 22, font: 'Arial', italics: true })]
  });
}

// Simple 2-column table helper
function simpleTable(headers, rows, colWidths) {
  const border = { style: BorderStyle.SINGLE, size: 1, color: '999999' };
  const borders = { top: border, bottom: border, left: border, right: border };

  const headerRow = new TableRow({
    children: headers.map((h, i) => new TableCell({
      borders,
      width: { size: colWidths[i], type: WidthType.DXA },
      shading: { fill: 'D5E8F0', type: ShadingType.CLEAR },
      margins: { top: 80, bottom: 80, left: 120, right: 120 },
      children: [new Paragraph({
        children: [new TextRun({ text: h, bold: true, size: 22, font: 'Arial' })]
      })]
    }))
  });

  const dataRows = rows.map(row => new TableRow({
    children: row.map((cell, i) => new TableCell({
      borders,
      width: { size: colWidths[i], type: WidthType.DXA },
      margins: { top: 80, bottom: 80, left: 120, right: 120 },
      children: [new Paragraph({
        children: [new TextRun({ text: cell, size: 22, font: 'Arial' })]
      })]
    }))
  }));

  return new Table({
    width: { size: CONTENT_W, type: WidthType.DXA },
    columnWidths: colWidths,
    rows: [headerRow, ...dataRows]
  });
}

// ─── Document ───────────────────────────────────────────────────────────────

const doc = new Document({
  numbering: {
    config: [
      {
        reference: 'bullets',
        levels: [{
          level: 0, format: LevelFormat.BULLET, text: '\u2022', alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } }
        }, {
          level: 1, format: LevelFormat.BULLET, text: '\u25E6', alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 1080, hanging: 360 } } }
        }]
      }
    ]
  },
  styles: {
    default: { document: { run: { font: 'Arial', size: 24 } } },
    paragraphStyles: [
      {
        id: 'Heading1', name: 'Heading 1', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 32, bold: true, font: 'Arial', color: '1F3864' },
        paragraph: { spacing: { before: 360, after: 180 }, outlineLevel: 0 }
      },
      {
        id: 'Heading2', name: 'Heading 2', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 28, bold: true, font: 'Arial', color: '2E74B5' },
        paragraph: { spacing: { before: 240, after: 120 }, outlineLevel: 1 }
      },
      {
        id: 'Heading3', name: 'Heading 3', basedOn: 'Normal', next: 'Normal', quickFormat: true,
        run: { size: 26, bold: true, font: 'Arial', color: '2E74B5' },
        paragraph: { spacing: { before: 200, after: 100 }, outlineLevel: 2 }
      }
    ]
  },
  sections: [{
    properties: {
      page: {
        size: { width: 11906, height: 16838 },
        margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 }
      }
    },
    children: [

      // ── Title Page ──────────────────────────────────────────────────────
      new Paragraph({
        alignment: AlignmentType.CENTER,
        spacing: { before: 2880, after: 240 },
        children: [new TextRun({ text: 'Sistemas Rob\u00F3ticos Aut\u00F3nomos 2025/2026', size: 28, bold: true, font: 'Arial' })]
      }),
      new Paragraph({
        alignment: AlignmentType.CENTER,
        spacing: { before: 120, after: 480 },
        children: [new TextRun({ text: 'Desenvolvimento de um rob\u00F4 aut\u00F3nomo para aplica\u00E7\u00F5es simples', size: 32, bold: true, font: 'Arial' })]
      }),
      new Paragraph({
        alignment: AlignmentType.CENTER,
        spacing: { before: 120, after: 120 },
        children: [new TextRun({ text: 'J\u00FAlia Ribeiro Baptista', size: 24, font: 'Arial' })]
      }),
      new Paragraph({
        alignment: AlignmentType.CENTER,
        spacing: { before: 60, after: 60 },
        children: [new TextRun({ text: '2025259589', size: 24, font: 'Arial' })]
      }),
      new Paragraph({
        alignment: AlignmentType.CENTER,
        spacing: { before: 60, after: 1440 },
        children: [new TextRun({ text: 'uc2025259589@student.uc.pt', size: 24, font: 'Arial' })]
      }),

      // ── 1. Introduction ─────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '1 Introdu\u00E7\u00E3o', bold: true, size: 32, font: 'Arial' })] }),

      para('A rob\u00F3tica m\u00F3vel constitui uma \u00E1rea de investiga\u00E7\u00E3o de crescente relev\u00E2ncia, com aplica\u00E7\u00F5es que v\u00E3o desde a log\u00EDstica industrial \u00E0 assist\u00EAncia dom\u00E9stica. Um dos desafios centrais desta \u00E1rea \u00E9 o desenvolvimento de sistemas capazes de navegar de forma aut\u00F3noma em ambientes estruturados ou semi-estruturados, combinando perce\u00E7\u00E3o sensorial, planeamento de trajet\u00F3rias e controlo de movimento.'),

      para('O presente trabalho insere-se no \u00E2mbito da disciplina de Sistemas Rob\u00F3ticos Aut\u00F3nomos e tem como objetivo o desenvolvimento progressivo de um sistema de navega\u00E7\u00E3o para um rob\u00F4 m\u00F3vel diferencial, o TurtleBot3 Burger. Ao longo do semestre, foram implementados e validados diferentes m\u00F3dulos fundamentais da rob\u00F3tica m\u00F3vel, designadamente:'),

      bullet('Controlo de movimento ponto-a-ponto e pose-a-pose;'),
      bullet('Seguimento de trajet\u00F3rias arbitr\u00E1rias;'),
      bullet('Planeamento global de caminhos com algoritmo A*;'),
      bullet('Navega\u00E7\u00E3o global com integra\u00E7\u00E3o de mapeamento LiDAR;'),
      bullet('Desvio local de obst\u00E1culos com VFF e VFH;'),
      bullet('Mapeamento bayesiano com navega\u00E7\u00E3o integrada;'),
      bullet('Localiza\u00E7\u00E3o por Filtro de Kalman Estendido (EKF) com corre\u00E7\u00E3o LiDAR.'),
      emptyLine(),

      para('Cada m\u00F3dulo foi desenvolvido de forma incremental e validado individualmente antes de ser integrado no sistema completo. O objetivo final \u00E9 a constru\u00E7\u00E3o de um rob\u00F4 m\u00F3vel aut\u00F3nomo capaz de navegar em ambientes estruturados de forma eficiente e segura, com localiza\u00E7\u00E3o probabilisticamente corrigida.'),

      // ── 2. Ambiente de Desenvolvimento ──────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '2 Ambiente de desenvolvimento', bold: true, size: 32, font: 'Arial' })] }),

      para('O ambiente de desenvolvimento foi configurado para permitir simula\u00E7\u00E3o rob\u00F3tica e controlo remoto, garantindo flexibilidade e reprodutibilidade experimental.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '2.1 Arquitetura de Execu\u00E7\u00E3o', bold: true, size: 28, font: 'Arial' })] }),

      para('O sistema foi dividido em dois n\u00EDveis funcionais que comunicam atrav\u00E9s do protocolo ROS:'),
      bullet('Host (MATLAB R2025b): respons\u00E1vel pela implementa\u00E7\u00E3o dos algoritmos de controlo, planeamento, mapeamento e localiza\u00E7\u00E3o (EKF).'),
      bullet('M\u00E1quina Virtual (ROS 1 Melodic + Gazebo 9): respons\u00E1vel pela simula\u00E7\u00E3o do rob\u00F4 e pela gest\u00E3o das comunica\u00E7\u00F5es ROS (publica\u00E7\u00E3o/subscri\u00E7\u00E3o de t\u00F3picos).'),
      emptyLine(),

      para('Esta separa\u00E7\u00E3o permite isolar o ambiente de simula\u00E7\u00E3o e testar algoritmos sem comprometer o sistema operativo do host, ao mesmo tempo que mant\u00E9m a compatibilidade com o hardware real.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '2.2 Especifica\u00E7\u00F5es do Sistema', bold: true, size: 28, font: 'Arial' })] }),

      simpleTable(
        ['Componente', 'Especifica\u00E7\u00E3o'],
        [
          ['Sistema Operativo Host', 'Ubuntu 22.04 (x86_64)'],
          ['Sistema Operativo VM', 'Ubuntu 18.04 (VirtualBox 7.2)'],
          ['Middleware Rob\u00F3tico', 'ROS 1 Melodic'],
          ['Simulador', 'Gazebo 9'],
          ['Ambiente de Desenvolvimento', 'MATLAB R2025b (com ROS Toolbox)'],
          ['Pacote TurtleBot3', 'TurtleBot3-v09d (fornecido pelo docente)'],
          ['Plataforma Rob\u00F3tica', 'TurtleBot3 Burger']
        ],
        [4500, 4526]
      ),
      caption('Quadro 1 - Especifica\u00E7\u00F5es do Sistema'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '2.3 Plataforma Rob\u00F3tica - TurtleBot3 Burger', bold: true, size: 28, font: 'Arial' })] }),

      para('O TurtleBot3 Burger \u00E9 um rob\u00F4 diferencial de baixo custo amplamente utilizado em ensino e investiga\u00E7\u00E3o. As suas principais caracter\u00EDsticas relevantes para este trabalho s\u00E3o:'),
      bullet('Tipo de locomo\u00E7\u00E3o: diferencial (duas rodas motrizes);'),
      bullet('Interface de controlo: t\u00F3pico ROS /cmd_vel (geometria Twist);'),
      bullet('Velocidade m\u00E1xima linear: ~0.18 m/s; angular: ~2.8 rad/s;'),
      bullet('Sensor LiDAR 2D: resolu\u00E7\u00E3o angular de 1\u00B0, alcance entre 0.1 m e 3.5 m;'),
      bullet('Dist\u00E2ncia entre rodas (wheelbase): 0.16 m; raio da roda: 0.033 m;'),
      bullet('Frequ\u00EAncia de controlo utilizada: entre 5 Hz e 50 Hz.'),
      emptyLine(),

      // ── 3. Requisitos ───────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '3 Requisitos', bold: true, size: 32, font: 'Arial' })] }),

      para('O sistema desenvolvido segue os requisitos definidos no plano da disciplina. O quadro 2 apresenta todos os requisitos.'),
      emptyLine(),

      simpleTable(
        ['Requisito', 'Descri\u00E7\u00E3o', 'Estado'],
        [
          ['R01', 'Setup ROS/Gazebo e Teleop b\u00E1sica', 'Implementado'],
          ['R02', 'Cinem\u00E1tica e Controlo Ponto-a-Ponto', 'Implementado'],
          ['R03', 'Seguimento Trajet\u00F3ria e Orienta\u00E7\u00E3o', 'Implementado'],
          ['R04', 'Algoritmo A* (L\u00F3gica e Matriz)', 'Implementado'],
          ['R05', 'Navega\u00E7\u00E3o Global (A* + Waypoints)', 'Implementado'],
          ['R06', 'Desvio de Obst\u00E1culos (VFF)', 'Implementado'],
          ['R07', 'Histogramas (VFH)', 'Implementado'],
          ['R08', 'Modelo Inverso Sensor', 'Implementado'],
          ['R09', 'Log-Odds e Grelha', 'Implementado'],
          ['R10', 'SLAM B\u00E1sico', 'Implementado'],
          ['R11', 'EKF Predi\u00E7\u00E3o', 'Implementado'],
          ['R12', 'EKF Corre\u00E7\u00E3o (LiDAR)', 'Implementado (parcial)'],
          ['R13', 'Integra\u00E7\u00E3o e Debug', 'Em progresso']
        ],
        [1500, 5526, 2000]
      ),
      caption('Quadro 2 - Requisitos do Projeto'),

      para('Os requisitos R01 a R11 foram implementados e validados, tanto em simula\u00E7\u00E3o como em ambiente real. O requisito R12, relativo \u00E0 corre\u00E7\u00E3o EKF com dados LiDAR, encontra-se implementado com a estrutura de observa\u00E7\u00E3o e inovar\u00E7\u00E3o funcional, sendo o passo de atualiza\u00E7\u00E3o (ekfUpdate) ainda objeto de ajuste e valida\u00E7\u00E3o. O requisito R13, de integra\u00E7\u00E3o e debug completo, est\u00E1 em progresso.'),

      // ── 4. Design e Arquitetura ──────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '4 Design e Arquitetura do Sistema', bold: true, size: 32, font: 'Arial' })] }),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '4.1 Vis\u00E3o Geral', bold: true, size: 28, font: 'Arial' })] }),

      para('O sistema foi concebido com uma arquitetura modular em camadas, onde cada m\u00F3dulo \u00E9 respons\u00E1vel por uma fun\u00E7\u00E3o espec\u00EDfica e pode ser desenvolvido e testado de forma independente. Toda a l\u00F3gica de controlo e planeamento \u00E9 implementada em MATLAB, comunicando com o simulador Gazebo (ou o rob\u00F4 real) atrav\u00E9s da interface ROS.'),

      para('O fluxo de dados processa-se da seguinte forma: o m\u00F3dulo de mapeamento LiDAR constr\u00F3i uma grelha de ocupa\u00E7\u00E3o a partir das leituras do sensor; o planeador global A* calcula o caminho \u00F3timo entre dois pontos nessa grelha; o controlador de seguimento de trajet\u00F3ria (PathTracking) executa o percurso calculado; os m\u00F3dulos de evita\u00E7\u00E3o local (VFF/VFH) desviam o rob\u00F4 de obst\u00E1culos imprevistos em tempo real; e o m\u00F3dulo EKF estima e corrige a pose do rob\u00F4 fundindo os encoders simulados com as leituras do LiDAR.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '4.2 Estrutura do Reposit\u00F3rio', bold: true, size: 28, font: 'Arial' })] }),

      paraRuns([
        'O c\u00F3digo-fonte completo est\u00E1 dispon\u00EDvel publicamente no reposit\u00F3rio: ',
        { text: 'https://github.com/JuhRBaptista/sra-assignments2.git', color: '2E74B5' },
        ', o qual se encontra organizado da seguinte forma:'
      ]),
      emptyLine(),
      bullet('controllers/ - controladores de movimento:', 0),
      bullet('PointToPointControl.m - controlo proporcional ponto-a-ponto;', 1),
      bullet('PoseToPoseControl.m - controlo em coordenadas polares para pose completa (x, y, \u03B8);', 1),
      bullet('keyboardControl.m - controle do rob\u00F4 via teleoper\u00E7\u00E3o;', 1),
      bullet('navigation/ \u2014 algoritmos de planeamento e evita\u00E7\u00E3o:', 0),
      bullet('aStar.m - implementa\u00E7\u00E3o do algoritmo A* com fun\u00E7\u00F5es auxiliares;', 1),
      bullet('mapBuildingWithTracking.m - constru\u00E7\u00E3o interativa de mapa e navega\u00E7\u00E3o aut\u00F3noma;', 1),
      bullet('PathTracking.m \u2014 seguimento de trajet\u00F3ria com suporte a VFF, VFH e EKF;', 1),
      bullet('VFF.m - Campo de For\u00E7as Virtuais para evita\u00E7\u00E3o de obst\u00E1culos;', 1),
      bullet('VFH.m - Histograma de Campo de For\u00E7as para navega\u00E7\u00E3o reativa;', 1),
      bullet('EKF/ \u2014 localiza\u00E7\u00E3o por Filtro de Kalman Estendido:', 0),
      bullet('EKF.m - fun\u00E7\u00E3o principal que integra predi\u00E7\u00E3o e corre\u00E7\u00E3o;', 1),
      bullet('ekfPredict.m - passo de predi\u00E7\u00E3o com modelo cinem\u00E1tico diferencial;', 1),
      bullet('ekfUpdate.m - passo de corre\u00E7\u00E3o com inovador de Kalman;', 1),
      bullet('g.m - modelo de observa\u00E7\u00E3o LiDAR (ray casting) e jacobiano;', 1),
      bullet('mapping/ - c\u00F3digo e fun\u00E7\u00F5es auxiliares para cria\u00E7\u00E3o do mapa e navega\u00E7\u00E3o bayesiana;', 0),
      bullet('utils/ \u2014 utilit\u00E1rios gerais (TurtleBot3.m, connectRobot.m, etc.);', 0),
      bullet('graphics/ \u2014 fun\u00E7\u00F5es de visualiza\u00E7\u00E3o (setupPlot.m, updatePlot.m, etc.);', 0),
      bullet('tests/ \u2014 scripts de teste para cada requisito;', 0),
      bullet('variables/ \u2014 mapas e caminhos pr\u00E9-calculados (.mat);', 0),
      bullet('maps/ \u2014 imagens de mapas de grelha e modelos 3D Gazebo.', 0),
      emptyLine(),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '4.3 Especifica\u00E7\u00F5es da Representa\u00E7\u00E3o do Mapa', bold: true, size: 28, font: 'Arial' })] }),

      para('A grelha de ocupa\u00E7\u00E3o que serve de base ao planeamento e ao mapeamento tem as seguintes caracter\u00EDsticas:'),
      bullet('Dimens\u00E3o: 350 \u00D7 350 c\u00E9lulas (para o ambiente House; 80 \u00D7 80 para testes sint\u00E9ticos);'),
      bullet('Resolu\u00E7\u00E3o: 5 cm por c\u00E9lula;'),
      bullet('Escala: 20 pixels/metro (para o ambiente house) ou 45 pixels/metro (para ambiente sint\u00E9tico);'),
      bullet('Conven\u00E7\u00E3o de ocupa\u00E7\u00E3o: 0 = livre, 1 = ocupado;'),
      bullet('Infla\u00E7\u00E3o de obst\u00E1culos: dilata\u00E7\u00E3o com raio igual ao raio do rob\u00F4 (robot_pixels = round(0.105 \u00D7 scale));'),
      bullet('Representa\u00E7\u00E3o probabil\u00EDstica (Bayesiana): log-odds com limites [\u22125.0, +5.0] e probabilidade inicial 0.5.'),
      emptyLine(),

      // ── 5. Verificações ─────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '5 Verifica\u00E7\u00F5es', bold: true, size: 32, font: 'Arial' })] }),

      para('Esta sec\u00E7\u00E3o descreve os procedimentos de verifica\u00E7\u00E3o definidos para cada m\u00F3dulo, apresentando os crit\u00E9rios de aceita\u00E7\u00E3o e os cen\u00E1rios de teste utilizados.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.1 Ambiente de desenvolvimento (R01)', bold: true, size: 28, font: 'Arial' })] }),
      para('A verifica\u00E7\u00E3o do ambiente de desenvolvimento baseou-se nos seguintes crit\u00E9rios:'),
      bullet('Inicializa\u00E7\u00E3o sem erros dos ambientes de simula\u00E7\u00E3o Empty e House no Gazebo;'),
      bullet('Execu\u00E7\u00E3o funcional dos scripts de demonstra\u00E7\u00E3o inclu\u00EDdos no pacote TurtleBot3-v09d;'),
      bullet('Comunica\u00E7\u00E3o bidirecional est\u00E1vel entre o MATLAB e o simulador (publica\u00E7\u00E3o em /cmd_vel e subscri\u00E7\u00E3o de /odom e /scan).'),
      emptyLine(),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.2 Controlo Ponto a Ponto (R02)', bold: true, size: 28, font: 'Arial' })] }),
      para('O controlo ponto a ponto do rob\u00F4 realizou-se a partir do c\u00E1lculo da dist\u00E2ncia euclidiana do rob\u00F4 em rela\u00E7\u00E3o ao objetivo, bem como o erro de orienta\u00E7\u00E3o. A lei de controlo \u00E9 definida pelas equa\u00E7\u00F5es 1 e 2.'),
      para('v = kV \u00D7 d(pose, target)                                                                              (1)'),
      para('w = kW \u00D7 \u0394\u03B8(pose, target)                                                                           (2)'),
      para('Onde kV e kW s\u00E3o os ganhos proporcionais de velocidade linear e angular, respectivamente. A verifica\u00E7\u00E3o efetuou-se a partir do teste do algoritmo nos ambientes do Gazebo Empty, House e Office. O quadro 3 apresenta as posi\u00E7\u00F5es inicial e final utilizadas para verifica\u00E7\u00E3o em cada ambiente.'),
      emptyLine(),

      simpleTable(
        ['Teste', 'Ambiente', 'Origem', 'Destino'],
        [
          ['1', 'Office', '[0, 1]', '[5, 1]'],
          ['2', 'Empty', '[0, 0]', '[5, 5]'],
          ['3', 'House', '[-1, 1]', '[-1, 4]']
        ],
        [1500, 2500, 2500, 2526]
      ),
      caption('Quadro 3 - Testes de Deslocamento do Rob\u00F4 para cada ambiente'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.3 Seguimento Trajet\u00F3ria e Orienta\u00E7\u00E3o (R03)', bold: true, size: 28, font: 'Arial' })] }),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.3.1 Seguimento de Trajet\u00F3ria Circular', bold: true, size: 26, font: 'Arial' })] }),
      para('A trajet\u00F3ria foi definida atrav\u00E9s de um conjunto discreto de pontos no plano (x,y), obtidos a partir das equa\u00E7\u00F5es param\u00E9tricas de um c\u00EDrculo de raio R=1m e centro (x_center, y_center), conforme as equa\u00E7\u00F5es 3.'),
      para('x = x_center + R\u00B7cos(\u03B8),    y = y_center + R\u00B7sin(\u03B8)                                    (3)'),
      para('Durante a execu\u00E7\u00E3o dos ciclos \u00E9 utilizado o controlo elaborado na primeira atividade, e o ponto alvo seguinte \u00E9 mudado de cada vez que o TurtleBot se encontra a uma dist\u00E2ncia suficientemente pequena do ponto alvo atual. Foi realizado um ajuste dos ganhos kp, ki e kw para garantir um movimento est\u00E1vel e reduzir oscila\u00E7\u00F5es.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.3.2 Controlo Pose-a-Pose', bold: true, size: 26, font: 'Arial' })] }),
      para('O deslocamento para uma pose arbitr\u00E1ria realizou-se atrav\u00E9s de um controlador baseado na representa\u00E7\u00E3o da posi\u00E7\u00E3o relativa do rob\u00F4 em coordenadas polares. Os par\u00E2metros utilizados s\u00E3o:'),
      bullet('\u03C1 (rho): Dist\u00E2ncia entre o rob\u00F4 e o ponto objetivo;'),
      bullet('\u03B1 (alpha): Diferen\u00E7a entre a orienta\u00E7\u00E3o atual do rob\u00F4 e a dire\u00E7\u00E3o do objetivo;'),
      bullet('\u03B2 (beta): Diferen\u00E7a entre a orienta\u00E7\u00E3o desejada no objetivo e a orienta\u00E7\u00E3o atual do rob\u00F4.'),
      para('\u03BD = k\u03C1 \u00D7 \u03C1                                                                                                    (4)'),
      para('\u03C9 = k\u03B1 \u00D7 \u03B1 + k\u03B2 \u00D7 \u03B2                                                                                 (5)'),
      para('A verifica\u00E7\u00E3o realizou-se em simula\u00E7\u00E3o e no ambiente real onde, em ambos, o rob\u00F4 deveria mover-se com sucesso da pose (0, 0, 0) para a pose (1, 1, \u03C0) e da pose (0, 0, 0) para a pose (3, 3, \u03C0/2).'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.4 Algoritmo A* (R04)', bold: true, size: 28, font: 'Arial' })] }),
      para('A implementa\u00E7\u00E3o do algoritmo A* no MATLAB segue a estrutura cl\u00E1ssica com listas aberta e fechada. O c\u00F3digo foi organizado numa fun\u00E7\u00E3o principal (aStar.m) com cinco fun\u00E7\u00F5es auxiliares:'),
      bullet('getManhattanDistance: heur\u00EDstica de dist\u00E2ncia de Manhattan;'),
      bullet('getMinimumFScore: sele\u00E7\u00E3o do n\u00F3 com menor f-score na lista aberta;'),
      bullet('getNeighbors: identifica\u00E7\u00E3o dos 8 vizinhos v\u00E1lidos (conectividade 8);'),
      bullet('getNeighborDistance: custo de transi\u00E7\u00E3o (1 para movimentos cardinais, \u221A2 para diagonais);'),
      bullet('reconstructPath: reconstru\u00E7\u00E3o do caminho \u00F3timo por retrocesso no mapa cameFrom.'),
      emptyLine(),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.5 Navega\u00E7\u00E3o Global (A* + Waypoints)', bold: true, size: 28, font: 'Arial' })] }),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.5.1 Constru\u00E7\u00E3o do mapa', bold: true, size: 26, font: 'Arial' })] }),
      para('Foi desenvolvido um algoritmo simples de constru\u00E7\u00E3o de mapa (mapBuilding.m), onde a cada itera\u00E7\u00E3o os pontos detetados pelo sensor s\u00E3o convertidos da frame do rob\u00F4 para coordenadas do mapa atrav\u00E9s da cinem\u00E1tica direta. O algoritmo foi testado no ambiente de simula\u00E7\u00E3o Gazebo House.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.5.2 Planeamento do Percurso', bold: true, size: 26, font: 'Arial' })] }),
      para('O planeamento do percurso realizou-se atrav\u00E9s do mapa constru\u00EDdo na etapa anterior e do algoritmo A*. Os obst\u00E1culos do mapa foram inflados de acordo com as dimens\u00F5es do rob\u00F4 para garantir aus\u00EAncia de colis\u00E3o. O resultado esperado \u00E9 o menor caminho v\u00E1lido poss\u00EDvel.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.5.3 Seguimento do Percurso', bold: true, size: 26, font: 'Arial' })] }),
      para('O caminho gerado pelo A* \u00E9 convertido de coordenadas de grelha para coordenadas m\u00E9tricas do mundo e passado como entrada ao controlador PathTracking. O resultado foi testado tanto em simula\u00E7\u00E3o como no ambiente real.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.6 Desvio de Obst\u00E1culos - VFF e VFH (R06 e R07)', bold: true, size: 28, font: 'Arial' })] }),
      para('O desvio de obst\u00E1culos envolve a implementa\u00E7\u00E3o de duas estrat\u00E9gias de planeamento local: Campo de For\u00E7as Virtuais (VFF) e Histograma de Campo de For\u00E7as (VFH). Ambos os algoritmos foram desenvolvidos e testados em quatro percursos distribu\u00EDdos por tr\u00EAs mapas sint\u00E9ticos:'),
      bullet('Mapa 1: percurso longo com obst\u00E1culos dispersos;'),
      bullet('Mapa 2: corredor estreito com obst\u00E1culos laterais densos;'),
      bullet('Mapa 3: curva acentuada com obst\u00E1culo em posi\u00E7\u00E3o central;'),
      bullet('Mapa 4 (beco sem sa\u00EDda): cen\u00E1rio onde se espera falha por m\u00EDnimo local.'),
      emptyLine(),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.6.1 Campo de For\u00E7as Virtuais (VFF)', bold: true, size: 26, font: 'Arial' })] }),
      para('O VFF (m\u00F3dulo VFF.m) modela o ambiente como um campo eletromagn\u00E9tico e calcula dois vetores de for\u00E7a. A for\u00E7a atrativa (Fa) \u00E9 dirigida ao waypoint atual, com magnitude proporcional \u00E0 dist\u00E2ncia normalizada. A for\u00E7a repulsiva (Fr) \u00E9 a soma das contribui\u00E7\u00F5es de todas as c\u00E9lulas ocupadas numa janela de pesquisa de 10\u00D710 c\u00E9lulas centrada no rob\u00F4:'),
      para('Fr = \u2212 kRep \u00D7 occValue / d\u00B2 \u00D7 (dx, dy) / d'),
      para('O vetor resultante F = Fa + Fr define a dire\u00E7\u00E3o do ponto-alvo intermedi\u00E1rio passado ao controlador PathTracking.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.6.2 Histograma de Campo de For\u00E7as (VFH)', bold: true, size: 26, font: 'Arial' })] }),
      para('O VFH (m\u00F3dulo VFH.m) discretiza o espa\u00E7o angular em torno do rob\u00F4 em setores de largura fixa e constr\u00F3i um histograma polar de densidade de obst\u00E1culos. O algoritmo de sele\u00E7\u00E3o de dire\u00E7\u00E3o identifica vales livres no histograma e seleciona o mais pr\u00F3ximo do alvo. O histograma \u00E9 suavizado com um filtro Gaussiano circular antes da decis\u00E3o.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.7 Mapeamento Bayesiano e Navega\u00E7\u00E3o Integrada (R08 a R10)', bold: true, size: 28, font: 'Arial' })] }),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.7.1 Modelo Inverso do Sensor (R08)', bold: true, size: 26, font: 'Arial' })] }),
      para('O modelo inverso do sensor traduz uma leitura do LiDAR numa atualiza\u00E7\u00E3o de probabilidade de ocupa\u00E7\u00E3o para as c\u00E9lulas do mapa. Para cada raio distinguem-se dois casos: raio com retorno (endpoint marcado como ocupado, c\u00E9lulas ao longo do raio marcadas como livres pelo algoritmo de Bresenham) e raio sem retorno (raio truncado ao range m\u00E1ximo de 2.0 m e c\u00E9lulas ao longo do mesmo marcadas como livres).'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.7.2 Log-Odds e Grelha de Ocupa\u00E7\u00E3o (R09)', bold: true, size: 26, font: 'Arial' })] }),
      para('A representa\u00E7\u00E3o probabil\u00EDstica do mapa utiliza a formula\u00E7\u00E3o log-odds. Os valores s\u00E3o limitados ao intervalo [l_min, l_max] = [\u22125.0, 5.0]. O quadro 4 resume os par\u00E2metros do m\u00F3dulo logOddsUpdate.m:'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Valor'],
        [
          ['l_occ (atualiza\u00E7\u00E3o ocupado)', '0.65'],
          ['l_free (atualiza\u00E7\u00E3o livre)', '-0.15'],
          ['l_min (clamping inferior)', '-5.0'],
          ['l_max (clamping superior)', '5.0'],
          ['Probabilidade inicial', '0.5 (l = 0)'],
          ['Range m\u00E1ximo LiDAR', '2.0 m'],
          ['Dimens\u00E3o do mapa', '80\u00D780 c\u00E9lulas (4\u00D74m)'],
          ['Resolu\u00E7\u00E3o da grelha', '5cm/c\u00E9lula (scale = 20 px/m)']
        ],
        [4500, 4526]
      ),
      caption('Quadro 4 - Par\u00E2metros do m\u00F3dulo logOddsUpdate.m'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.7.3 SLAM B\u00E1sico com Navega\u00E7\u00E3o Integrada (R10)', bold: true, size: 26, font: 'Arial' })] }),
      para('O m\u00F3dulo R10 integra o mapeamento bayesiano (R08 + R09) com o controlador PathTracking e os algoritmos de desvio de obst\u00E1culos VFF e VFH, atrav\u00E9s da fun\u00E7\u00E3o mapBuildingWithTracking.m. A localiza\u00E7\u00E3o \u00E9 baseada exclusivamente em odometria \u2014 sem corre\u00E7\u00E3o de pose \u2014 sendo esta a principal limita\u00E7\u00E3o identificada e endere\u00E7ada pelos m\u00F3dulos EKF (R11\u2013R13).'),

      // ── New EKF section ──────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '5.8 Localiza\u00E7\u00E3o por Filtro de Kalman Estendido (R11 a R13)', bold: true, size: 28, font: 'Arial' })] }),

      para('Os m\u00F3dulos R11 a R13 implementam um Filtro de Kalman Estendido (EKF) para corre\u00E7\u00E3o da posi\u00E7\u00E3o do rob\u00F4 em tempo real, endere\u00E7ando diretamente o drift odom\u00E9trico identificado como principal limita\u00E7\u00E3o no m\u00F3dulo R10. O EKF est\u00E1 integrado no controlador PathTrackingControl.m atrav\u00E9s da flag params.ekf = true.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.8.1 EKF - Passo de Predi\u00E7\u00E3o (R11)', bold: true, size: 26, font: 'Arial' })] }),
      para('O passo de predi\u00E7\u00E3o (ekfPredict.m) estima a nova pose do rob\u00F4 a partir dos incrementos de encoder, utilizando o modelo cinem\u00E1tico diferencial. O modelo de movimento \u00E9 n\u00E3o linear e o EKF lineariza-o atrav\u00E9s dos jacobianos Fp (em rela\u00E7\u00E3o \u00E0 pose) e Fn (em rela\u00E7\u00E3o ao ru\u00EDdo dos encoders):'),
      bullet('Wheelbase: L = 0.16 m;'),
      bullet('Ganhos de ru\u00EDdo de encoder: kr = kl = 0.001;'),
      bullet('Ru\u00EDdo de processo adicional: Q = diag([0.001, 0.001, 0.0001]).'),
      emptyLine(),
      para('A atualiza\u00E7\u00E3o da covari\u00E2ncia segue a forma padr\u00E3o:'),
      para('Cp = Fp \u00D7 Cp \u00D7 Fp\u1D40 + Fn \u00D7 Cn \u00D7 Fn\u1D40 + Q'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.8.2 Modelo de Observa\u00E7\u00E3o LiDAR (R12)', bold: true, size: 26, font: 'Arial' })] }),
      para('O modelo de observa\u00E7\u00E3o (g.m) simula a leitura esperada de cada raio LiDAR para uma dada pose estimada, realizando ray casting sobre o mapa de ocupa\u00E7\u00E3o probabil\u00EDstico com passo de 0.01 m. Uma c\u00E9lula \u00E9 considerada ocupada quando a sua probabilidade \u00E9 \u2265 0.6. O jacobiano Jg da fun\u00E7\u00E3o de observa\u00E7\u00E3o em rela\u00E7\u00E3o \u00E0 pose \u00E9:'),
      para('Jg = [\u2212cos(\u03B8_beam), \u2212sin(\u03B8_beam), 0]'),
      para('onde \u03B8_beam = pose(3) + \u00E2ngulo_raio. Quando n\u00E3o se deteta hit, o jacobiano \u00E9 zero (raio ignorado na atualiza\u00E7\u00E3o).'),
      para('O passo de corre\u00E7\u00E3o utiliza um mecanismo de gating para rejeitar associa\u00E7\u00F5es improb\u00E1veis, aceitando apenas os raios cuja inovao normalizada v\u00E9rifica:'),
      para('(v_i\u00B2 / S_i) \u2264 e\u00B2,   com e = 2'),
      para('onde S_i = Jg \u00D7 Cp \u00D7 Jg\u1D40 + r_i \u00E9 a covari\u00E2ncia da inovao e r_i = (0.035 \u00D7 z_i)\u00B2 \u00E9 o ru\u00EDdo de medida proporcional \u00E0 dist\u00E2ncia medida. Esta escolha modela o facto de o LiDAR ser proporcionalmente menos preciso a longas dist\u00E2ncias. Os raios aceites s\u00E3o empilhados nas matrizes V (inova\u00E7\u00F5es), G (jacobianos) e R (covari\u00E2ncias), para posterior atualiza\u00E7\u00E3o em batch.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.8.3 EKF - Passo de Corre\u00E7\u00E3o (R12)', bold: true, size: 26, font: 'Arial' })] }),
      para('O passo de corre\u00E7\u00E3o (ekfUpdate.m) atualiza a pose estimada e a covari\u00E2ncia utilizando o ganho de Kalman calculado a partir do conjunto de observa\u00E7\u00F5es LiDAR aceites:'),
      para('S = G \u00D7 Cp \u00D7 G\u1D40 + R_mat'),
      para('K = Cp \u00D7 G\u1D40 / S'),
      para('p = p + K \u00D7 V'),
      para('A covari\u00E2ncia \u00E9 atualizada pela forma estabilizada de Joseph para garantir simetria e semi-definidade positiva:'),
      para('Cp = (I \u2212 K\u00D7G) \u00D7 Cp \u00D7 (I \u2212 K\u00D7G)\u1D40 + K \u00D7 R_mat \u00D7 K\u1D40'),
      para('O \u00E2ngulo \u03B8 \u00E9 normalizado ap\u00F3s cada atualiza\u00E7\u00E3o pelo intervalo [\u2212\u03C0, \u03C0].'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '5.8.4 Integra\u00E7\u00E3o e Debug (R13)', bold: true, size: 26, font: 'Arial' })] }),
      para('A integra\u00E7\u00E3o completa do EKF com o PathTrackingControl encontra-se implementada no script EKF_Validation.m. O ciclo de controlo com EKF ativado opera da seguinte forma:'),
      bullet('Leitura dos incrementos de encoder com ru\u00EDdo Gaussiano (readEncodersWithNoise, \u03C3 = 0.002 m);'),
      bullet('Passo de predi\u00E7\u00E3o: ekfPredict atualiza a pose e a covari\u00E2ncia;'),
      bullet('Leitura do LiDAR e c\u00E1lculo das observa\u00E7\u00F5es esperadas por ray casting (g.m);'),
      bullet('Filtragem por gating e empilhamento das inova\u00E7\u00F5es v\u00E1lidas;'),
      bullet('Passo de corre\u00E7\u00E3o: ekfUpdate atualiza a pose com as observa\u00E7\u00F5es LiDAR aceites;'),
      bullet('Visualiza\u00E7\u00E3o em tempo real da el\u00EDpse de covari\u00E2ncia (95%), da trajet\u00F3ria EKF e da ground truth do Gazebo.'),
      emptyLine(),
      para('Crit\u00E9rios de aceita\u00E7\u00E3o: (1) a el\u00EDpse de covari\u00E2ncia deve encolher ap\u00F3s associa\u00E7\u00F5es LiDAR bem-sucedidas; (2) a trajet\u00F3ria EKF deve divergir menos da ground truth do que a odometria pura; (3) aus\u00EAncia de colapso num\u00E9rico da covari\u00E2ncia ao longo de trajet\u00F3rias longas.'),

      simpleTable(
        ['Par\u00E2metro', 'Valor'],
        [
          ['Wheelbase (L)', '0.16 m'],
          ['Ganho de ru\u00EDdo encoder (kr = kl)', '0.001'],
          ['Q (incerteza de posi\u00E7\u00E3o)', '0.001 m\u00B2'],
          ['Q (incerteza de \u00E2ngulo)', '0.0001 rad\u00B2'],
          ['Ru\u00EDdo de medida LiDAR r_i', '(0.035 \u00D7 z_i)\u00B2'],
          ['Gate size (e)', '2'],
          ['Passo ray casting', '0.01 m'],
          ['Range m\u00E1ximo LiDAR (mapParams)', '2.0 m'],
          ['Std ru\u00EDdo encoder simulado', '0.002 m'],
          ['Frequ\u00EAncia de controlo', '50 Hz']
        ],
        [5000, 4026]
      ),
      caption('Quadro 5 - Par\u00E2metros do m\u00F3dulo EKF'),

      // ── 6. Validações ───────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '6 Valida\u00E7\u00F5es', bold: true, size: 32, font: 'Arial' })] }),

      para('Esta sec\u00E7\u00E3o apresenta os resultados obtidos para cada m\u00F3dulo, discutindo o desempenho observado tanto em simula\u00E7\u00E3o como em ambiente real. Todos os v\u00EDdeos dos testes realizados est\u00E3o dispon\u00EDveis num drive partilhado. Relativamente ao c\u00F3digo desenvolvido, todos os scripts de valida\u00E7\u00E3o encontram-se na pasta "/matlab/validations".'),
      paraRuns([
        'Link de acesso ao drive: ',
        { text: 'SRA_Assignments_Videos', color: '2E74B5' }
      ]),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.1 Ambiente de Desenvolvimento (R01)', bold: true, size: 28, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o do sistema demonstrou que o MATLAB foi corretamente instalado e reconhece os toolboxes ROS, o Gazebo executa ambientes de simula\u00E7\u00E3o com o TurtleBot3 sem falhas, os scripts fornecidos podem ser executados na VM e a instala\u00E7\u00E3o dos mapas 3D foi conclu\u00EDda com sucesso.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.2 Controlo Ponto-a-Ponto (R02)', bold: true, size: 28, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o do Controlo Ponto a Ponto foi feita pelo script "W1/PointToPoint_Validation.m". O quadro 6 apresenta os par\u00E2metros utilizados.'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Descri\u00E7\u00E3o', 'Valor'],
        [
          ['kV', 'Ganho da velocidade linear', '2'],
          ['kW', 'Ganho da velocidade angular', '1'],
          ['vMax', 'Velocidade Linear M\u00E1xima', '0.18'],
          ['wMax', 'Velocidade Angular M\u00E1xima', '2.8'],
          ['rate', 'Frequ\u00EAncia de Itera\u00E7\u00E3o', '5'],
          ['maxIterations', 'Quantidade m\u00E1xima de itera\u00E7\u00F5es', '150'],
          ['toleranceError', 'Dist\u00E2ncia m\u00EDnima de paragem', '0.2']
        ],
        [2000, 4000, 3026]
      ),
      caption('Quadro 6 - Par\u00E2metros de valida\u00E7\u00E3o do Controlo Ponto a Ponto'),

      para('Os resultados demonstraram um deslocamento consistente e est\u00E1vel em todos os ambientes testados. O rob\u00F4 convergiu progressivamente para a posi\u00E7\u00E3o desejada sem oscila\u00E7\u00F5es significativas. Em todos os casos, a trajet\u00F3ria descrita pelo rob\u00F4 aproximou-se de uma linha reta entre os pontos de origem e destino.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.3 Seguimento Trajet\u00F3ria e Orienta\u00E7\u00E3o (R03)', bold: true, size: 28, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o dos algoritmos de seguimento de trajet\u00F3ria e orienta\u00E7\u00E3o realizou-se atrav\u00E9s de testes em simula\u00E7\u00E3o no Gazebo e em ambiente real. O controlador implementado demonstrou ser capaz de conduzir o rob\u00F4 at\u00E9 a pose desejada de forma est\u00E1vel e com boa precis\u00E3o.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.3.1 Controlo Pose-to-Pose', bold: true, size: 26, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o do controlo Pose-to-Pose realizou-se atrav\u00E9s do script "W1/PoseToPose_Validation.m". Os par\u00E2metros foram testados empiricamente at\u00E9 encontrar uma combina\u00E7\u00E3o satisfat\u00F3ria.'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Descri\u00E7\u00E3o', 'Valor'],
        [
          ['kpRho', 'Ganho linear', '1.5'],
          ['kpAlpha', 'Ganho da orienta\u00E7\u00E3o', '2'],
          ['kpBeta', 'Ganho da orienta\u00E7\u00E3o final', '-0.4'],
          ['vMax', 'Velocidade Linear M\u00E1xima', '0.18'],
          ['wMax', 'Velocidade Angular M\u00E1xima', '2.8'],
          ['rate', 'Frequ\u00EAncia de Itera\u00E7\u00E3o', '5'],
          ['maxIterations', 'Quantidade m\u00E1xima de itera\u00E7\u00F5es', '500'],
          ['toleranceErrorDist', 'Dist\u00E2ncia m\u00EDnima de paragem', '0.4'],
          ['toleranceErrorAngle', 'Varia\u00E7\u00E3o angular m\u00EDnima de paragem', '0.05']
        ],
        [2500, 4000, 2526]
      ),
      caption('Quadro 7 - Par\u00E2metros de valida\u00E7\u00E3o do Controlo Pose a Pose'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.3.2 Controlo de seguimento de trajet\u00F3ria', bold: true, size: 26, font: 'Arial' })] }),
      para('O controlo de seguimento de trajet\u00F3ria foi testado no seguimento de uma trajet\u00F3ria circular, atrav\u00E9s do script "W1/PathTracking_Validation.m". Os par\u00E2metros foram testados empiricamente at\u00E9 obter resultados satisfat\u00F3rios.'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Descri\u00E7\u00E3o', 'Valor'],
        [
          ['kv', 'Ganho linear proporcional', '2'],
          ['ki', 'Ganho linear integral', '0.1'],
          ['ks', 'Ganho de orienta\u00E7\u00E3o', '3.0'],
          ['distance', 'Dist\u00E2ncia desejada', '0.1'],
          ['vMax', 'Velocidade Linear M\u00E1xima', '0.18'],
          ['wMax', 'Velocidade Angular M\u00E1xima', '2.8'],
          ['rate', 'Frequ\u00EAncia de Itera\u00E7\u00E3o', '5'],
          ['dt', 'Time step', '0.05'],
          ['T', 'Tempo m\u00E1ximo de execu\u00E7\u00E3o', '60'],
          ['toleranceError', 'Dist\u00E2ncia m\u00EDnima de paragem', '0.2']
        ],
        [2500, 4000, 2526]
      ),
      caption('Quadro 8 - Par\u00E2metros de valida\u00E7\u00E3o do Controlo de Seguimento de Trajet\u00F3ria'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.4 Algoritmo A* (L\u00F3gica e Matriz)', bold: true, size: 28, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o do algoritmo foi feita por dois scripts: "W2/AStar_SimpleMap_Validation.m" e "W2/AStar_ComplexMap_Validation.m". Inicialmente foram utilizados mapas pequenos (4\u00D74) para validar o funcionamento do algoritmo e facilitar a verifica\u00E7\u00E3o manual. Posteriormente foram gerados mapas de dimens\u00E3o 80\u00D780 com densidades de ocupa\u00E7\u00E3o de 10%, 25% e 40%. Os resultados demonstraram que o algoritmo foi capaz de encontrar trajet\u00F3rias v\u00E1lidas sempre que existia um caminho poss\u00EDvel.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.5 Navega\u00E7\u00E3o Global (A* + Waypoints)', bold: true, size: 28, font: 'Arial' })] }),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.5.1 Constru\u00E7\u00E3o do mapa', bold: true, size: 26, font: 'Arial' })] }),
      para('O mapa do local apresentou formato e dimens\u00F5es pr\u00F3ximas ao do ambiente de simula\u00E7\u00E3o, apesar de ainda possuir erros e ru\u00EDdos devido \u00E0 sua implementa\u00E7\u00E3o mais simples.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.5.2 Planeamento do Percurso', bold: true, size: 26, font: 'Arial' })] }),
      para('Foram realizados dois testes para avalia\u00E7\u00E3o do planeamento de percurso, onde em ambos o rob\u00F4 devia atravessar dois c\u00F4modos da casa. Os resultados obtidos foram trajet\u00F3rias vi\u00E1veis com o caminho otimizado.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.5.3 Seguimento do Percurso', bold: true, size: 26, font: 'Arial' })] }),
      para('As trajet\u00F3rias foram validadas na simula\u00E7\u00E3o e no ambiente real, onde em ambos os casos o rob\u00F4 foi capaz de percorrer a trajet\u00F3ria com sucesso sem colidir com os obst\u00E1culos. Os par\u00E2metros de Path Tracking utilizados foram: kpV = 2.5, kiV = 0.1, kpW = 1.0.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.6 Desvio de Obst\u00E1culos - VFF e VFH (R06 e R07)', bold: true, size: 28, font: 'Arial' })] }),
      para('Os scripts utilizados para validar os algoritmos de desvio de obst\u00E1culos foram "W3/VFF_Validation.m" e "W3/VFH_Validation.m". Ambos os m\u00E9todos foram integrados ao controlador de seguimento de trajet\u00F3ria.'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.6.1 Resultados do VFF', bold: true, size: 26, font: 'Arial' })] }),
      para('O VFF demonstrou capacidade de seguir percursos em ambientes com obst\u00E1culos sem colis\u00F5es para os mapas 1, 2 e 3. Contudo, foi observado o comportamento t\u00EDpico deste algoritmo: ondula\u00E7\u00F5es na trajet\u00F3ria resultantes das for\u00E7as repulsivas de obst\u00E1culos laterais, e incapacidade de escapar a becos sem sa\u00EDda (mapa 4).'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Descri\u00E7\u00E3o', 'Valor'],
        [
          ['kAtt', 'For\u00E7a atrativa', '1.0'],
          ['kRep', 'For\u00E7a repulsiva', '1.0'],
          ['windowSize', 'Tamanho da janela de busca', '10\u00D710']
        ],
        [2000, 4000, 3026]
      ),
      caption('Quadro 9 - Par\u00E2metros do VFF'),

      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun({ text: '6.6.2 Histogramas VFH', bold: true, size: 26, font: 'Arial' })] }),
      para('O VFH demonstrou desempenho superior ao VFF em termos de suavidade de trajet\u00F3ria. O rob\u00F4 completou com sucesso os mapas 1, 2 e 3 sem colis\u00F5es. No mapa 4 o VFH tamb\u00E9m falhou, confirmando que ambos os algoritmos de planeamento local n\u00E3o resolvem cen\u00E1rios com m\u00EDnimos locais. A trajet\u00F3ria VFH no primeiro mapa foi adicionalmente validada no TurtleBot3 real.'),
      emptyLine(),

      simpleTable(
        ['Par\u00E2metro', 'Descri\u00E7\u00E3o', 'Valor'],
        [
          ['numSectors', 'N\u00FAmero de se\u00E7\u00F5es do histograma', '72'],
          ['sectorWidth', 'Varia\u00E7\u00E3o angular de cada setor', '\u03C0/36 rad (~5\u00B0)'],
          ['smoothSigma', 'Par\u00E2metro de suaviza\u00E7\u00E3o gaussiana', '1.5'],
          ['threshold', 'Threshold para histograma bin\u00E1rio', '0.6'],
          ['valleyMinWidth', 'Largura m\u00EDnima para o vale ser considerado livre', '18'],
          ['windowSize', 'Tamanho da janela de busca', '10']
        ],
        [2500, 4000, 2526]
      ),
      caption('Quadro 10 - Par\u00E2metros do VFH'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.7 Mapeamento Bayesiano e Navega\u00E7\u00E3o Integrada (R08 a R10)', bold: true, size: 28, font: 'Arial' })] }),
      para('A integra\u00E7\u00E3o do mapeamento bayesiano com o PathTracking e o desvio de obst\u00E1culos foi validada em dois ambientes (YMAP e LCMAP) com cada um dos algoritmos de desvio (VFF e VFH), obtendo quatro configura\u00E7\u00F5es de teste. Em todas as configura\u00E7\u00F5es, o rob\u00F4 completou o percurso pr\u00E9-planeado sem colidir com obst\u00E1culos, construindo simultaneamente uma representa\u00E7\u00E3o do ambiente.'),
      para('As principais observa\u00E7\u00F5es s\u00E3o: fidelidade geom\u00E9trica preservada em ambos os ambientes; VFH gera mapas ligeiramente mais definidos do que o VFF devido \u00E0 maior suavidade dos movimentos; e a grelha de 80\u00D780 c\u00E9lulas \u00E9 suficiente para os ambientes testados. O algoritmo foi igualmente validado em ambiente real com resultados satisfat\u00F3rios, sendo o drift odom\u00E9trico a principal limita\u00E7\u00E3o observada.'),

      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun({ text: '6.8 Localiza\u00E7\u00E3o por EKF (R11 a R13)', bold: true, size: 28, font: 'Arial' })] }),
      para('A valida\u00E7\u00E3o do EKF realizou-se atrav\u00E9s do script "W5/EKF_Validation.m", utilizando o ambiente rmap com navega\u00E7\u00E3o VFH. O sistema opera a 50 Hz com o EKF a correr em cada itera\u00E7\u00E3o do ciclo de controlo.'),

      para('O script de valida\u00E7\u00E3o habilita a visualiza\u00E7\u00E3o simult\u00E2nea de: (1) trajet\u00F3ria estimada pelo EKF; (2) ground truth obtida do Gazebo via /gazebo/model_states; (3) el\u00EDpse de covari\u00E2ncia a 95%; e (4) histograma VFH. Esta combina\u00E7\u00E3o permite avaliar qualitativamente a qualidade da localiza\u00E7\u00E3o ao longo da trajet\u00F3ria.'),

      para('O passo de corre\u00E7\u00E3o (ekfUpdate) encontra-se implementado mas o seu uso est\u00E1 comentado na fun\u00E7\u00E3o EKF.m, uma vez que a acumula\u00E7\u00E3o e filtragem das observa\u00E7\u00F5es por gating est\u00E3o conclu\u00EDdas e o m\u00F3dulo encontra-se em fase de ajuste de par\u00E2metros e debug. Os crit\u00E9rios de aceita\u00E7\u00E3o (redu\u00E7\u00E3o da covari\u00E2ncia ap\u00F3s associa\u00E7\u00F5es, menor diverg\u00EAncia face \u00E0 ground truth, estabilidade num\u00E9rica) ser\u00E3o verificados na valida\u00E7\u00E3o final do m\u00F3dulo R13.'),

      // ── 7. Conclusão ────────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '7 Conclus\u00E3o', bold: true, size: 32, font: 'Arial' })] }),

      para('O trabalho desenvolvido permitiu a implementa\u00E7\u00E3o de um sistema rob\u00F3tico aut\u00F3nomo baseado numa arquitetura modular, integrando perce\u00E7\u00E3o, planeamento, controlo e localiza\u00E7\u00E3o. O sistema demonstrou ser capaz de construir representa\u00E7\u00F5es do ambiente, planear trajet\u00F3rias seguras e executar o movimento do rob\u00F4 de forma consistente, tanto em simula\u00E7\u00E3o como em ambiente real.'),

      para('A utiliza\u00E7\u00E3o do MATLAB em conjunto com o ROS e o Gazebo revelou-se uma combina\u00E7\u00E3o eficaz para o desenvolvimento e valida\u00E7\u00E3o de algoritmos de navega\u00E7\u00E3o aut\u00F3noma, permitindo iterar rapidamente sobre os par\u00E2metros dos controladores e visualizar os resultados em tempo real.'),

      para('A compara\u00E7\u00E3o entre VFF e VFH evidenciou as diferen\u00E7as pr\u00E1ticas entre os dois algoritmos: o VFF \u00E9 mais simples mas produz trajet\u00F3rias mais oscilat\u00F3rias, enquanto o VFH produz movimentos mais suaves gra\u00E7as ao mecanismo de sele\u00E7\u00E3o por vale. Ambos falham em cen\u00E1rios com m\u00EDnimos locais, o que refor\u00E7a a import\u00E2ncia da combina\u00E7\u00E3o de planeamento global (A*) com evita\u00E7\u00E3o local reativa.'),

      para('A implementa\u00E7\u00E3o do mapeamento bayesiano com log-odds (R08 e R09) permitiu ao rob\u00F4 construir uma representa\u00E7\u00E3o probabil\u00EDstica do ambiente a partir das leituras do LiDAR, sem conhecimento pr\u00E9vio do espa\u00E7o. A integra\u00E7\u00E3o deste m\u00F3dulo com o PathTracking e os algoritmos de desvio de obst\u00E1culos (R10) resultou num sistema de SLAM b\u00E1sico funcional. A principal limita\u00E7\u00E3o identificada \u00E9 a depend\u00EAncia exclusiva da odometria para localiza\u00E7\u00E3o.'),

      para('Os m\u00F3dulos EKF (R11-R13) foram desenvolvidos para endere\u00E7ar diretamente essa limita\u00E7\u00E3o. A fase de predi\u00E7\u00E3o (R11) e o modelo de observa\u00E7\u00E3o LiDAR com gating (R12) encontram-se implementados. A completa ativa\u00E7\u00E3o do passo de corre\u00E7\u00E3o e a valida\u00E7\u00E3o experimental comparativa entre a trajet\u00F3ria EKF e a ground truth do Gazebo constituem o trabalho em curso (R13).'),

      // ── 8. Referências ──────────────────────────────────────────────────
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun({ text: '8 Refer\u00EAncias', bold: true, size: 32, font: 'Arial' })] }),
      para('Material da disciplina Sistemas Rob\u00F3ticos Aut\u00F3nomos, Universidade de Coimbra.')
    ]
  }]
});

Packer.toBuffer(doc).then(buffer => {
  fs.writeFileSync('RelatorioE9_SRA_JuliaRibeiroBaptista_updated.docx', buffer);
  console.log('Done');
});
