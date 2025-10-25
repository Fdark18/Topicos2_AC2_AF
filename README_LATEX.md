# Documentação Técnica em LaTeX - Sistema Embarcado de Tempo Real

Este diretório contém a documentação técnica completa do projeto em formato LaTeX, seguindo as normas ABNT e o modelo da UniFacens.

## 📁 Estrutura de Arquivos

```
mvp_freertos/
├── trabalho_academico.tex          # Arquivo principal do documento
├── references.bib                  # Referências bibliográficas
├── partes/
│   ├── pre_textuais/               # Elementos pré-textuais
│   │   ├── resumo.tex
│   │   ├── lista_figuras.tex
│   │   ├── lista_tabelas.tex
│   │   ├── lista_quadros.tex
│   │   ├── lista_siglas.tex
│   │   ├── sumario.tex
│   │   ├── agradecimentos.tex
│   │   ├── epigrafe.tex
│   │   ├── ficha_catalografica.tex
│   │   └── folha_aprovacao.tex
│   │
│   └── textuais/                   # Elementos textuais
│       ├── introducao.tex
│       ├── fundamentacao_teorica.tex
│       ├── configuracao_projeto.tex
│       ├── desenvolvimento.tex
│       ├── analise_desempenho.tex
│       ├── resultados.tex
│       └── conclusao.tex
│
└── lib/                            # Bibliotecas LaTeX (necessárias)
    ├── unifacens.sty
    └── url6023.sty
```

## 📋 Conteúdo do Documento

### Elementos Pré-Textuais
- Capa (gerada automaticamente)
- Folha de rosto
- Ficha catalográfica (a ser preenchida pela biblioteca)
- Folha de aprovação
- Agradecimentos
- Epígrafe
- Resumo (em português)
- Lista de Figuras
- Lista de Tabelas
- Lista de Quadros
- Lista de Siglas e Abreviaturas
- Sumário

### Elementos Textuais
1. **Introdução**
   - Contextualização
   - Objetivos (Geral e Específicos)
   - Justificativa
   - Organização do Documento

2. **Fundamentação Teórica**
   - Sistemas de Tempo Real
   - FreeRTOS
   - Rate Monotonic Scheduling
   - Thread Master de Emergência
   - Microcontroladores STM32

3. **Configuração do Projeto**
   - Especificações de Hardware
   - Configuração do STM32CubeIDE
   - Configuração de Periféricos (GPIO, PWM, UART)
   - Configuração do FreeRTOS

4. **Desenvolvimento do Projeto**
   - Arquitetura do Sistema
   - Implementação dos Drivers (USART, GPIO, syscalls)
   - Implementação dos Nós (Safety, Conveyor, Camera, Piston)
   - Sistema de Comunicação
   - Métricas de Tempo Real

5. **Análise de Desempenho**
   - Análise de WCET
   - Análise de Schedulability (Rate Monotonic)
   - Análise de Latências
   - Otimizações Implementadas

6. **Resultados e Discussão**
   - Testes Funcionais
   - Testes de Tempo Real
   - Testes de Emergência
   - Discussão dos Resultados

7. **Considerações Finais**
   - Conclusões
   - Trabalhos Futuros

### Elementos Pós-Textuais
- Referências Bibliográficas
- Índice Remissivo (opcional)

## 🔧 Requisitos para Compilação

### Opção 1: Overleaf (Recomendado para iniciantes)

1. Acesse [www.overleaf.com](https://www.overleaf.com)
2. Crie uma conta (gratuita)
3. Crie um novo projeto: `New Project → Upload Project`
4. Faça upload do arquivo ZIP contendo todos os arquivos
5. Compile automaticamente (botão `Recompile`)

**Observação:** Você precisará criar/fazer upload dos arquivos da pasta `lib/` (unifacens.sty e url6023.sty). Se não tiver acesso a eles, remova as linhas correspondentes no preâmbulo do documento.

### Opção 2: Instalação Local

#### Windows

1. Instalar **MiKTeX**: https://miktex.org/download
2. Instalar **TeXstudio**: https://www.texstudio.org/
3. Abrir `trabalho_academico.tex` no TeXstudio
4. Compilar: `Tools → Commands → PDFLaTeX` (F5)
5. Gerar bibliografia: `Tools → Commands → BibTeX` (F8)
6. Compilar novamente (2x) para resolver referências

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install texlive-full texstudio
cd mvp_freertos
pdflatex trabalho_academico.tex
bibtex trabalho_academico
pdflatex trabalho_academico.tex
pdflatex trabalho_academico.tex
```

#### macOS

```bash
brew install --cask mactex
brew install --cask texstudio
cd mvp_freertos
pdflatex trabalho_academico.tex
bibtex trabalho_academico.tex
pdflatex trabalho_academico.tex
pdflatex trabalho_academico.tex
```

## 📝 Como Compilar

### Método 1: Usando PDFLaTeX (Recomendado)

```bash
cd mvp_freertos

# Primeira compilação
pdflatex trabalho_academico.tex

# Gerar bibliografia
bibtex trabalho_academico

# Segunda compilação (resolver referências)
pdflatex trabalho_academico.tex

# Terceira compilação (resolver sumário e referências cruzadas)
pdflatex trabalho_academico.tex
```

O arquivo PDF será gerado como `trabalho_academico.pdf`.

### Método 2: Usando Makefile (se disponível)

```bash
make all      # Compila tudo
make clean    # Remove arquivos temporários
make view     # Abre o PDF gerado
```

### Método 3: Script de Compilação

Crie um arquivo `compile.sh`:

```bash
#!/bin/bash
pdflatex trabalho_academico.tex
bibtex trabalho_academico
pdflatex trabalho_academico.tex
pdflatex trabalho_academico.tex
echo "Compilação concluída! PDF gerado: trabalho_academico.pdf"
```

Execute:
```bash
chmod +x compile.sh
./compile.sh
```

## ✏️ Personalizando o Documento

### Alterar Informações da Capa

Edite o arquivo `trabalho_academico.tex`, seção de metadados:

```latex
\coordenadoria{Coordenadoria de Engenharia de Computação}
\titulo{Sistema Embarcado de Tempo Real...}
\integranteum{[Seu Nome]}
\integrantedois{[Nome do Colega 2]}
\integrantetres{[Nome do Colega 3]}
\local{Sorocaba/SP}
\data{2025}
\orientador{[Nome do Professor]}
```

### Adicionar Figuras

```latex
\begin{figure}[htb]
\centering
\caption{Descrição da figura}
\label{fig:minha_figura}
\includegraphics[width=0.8\textwidth]{images/minha_imagem.png}
\fonte{Elaborado pelo autor.}
\end{figure}
```

Coloque as imagens na pasta `images/`.

### Adicionar Tabelas

```latex
\begin{table}[htb]
\centering
\caption{Título da tabela}
\label{tab:minha_tabela}
\begin{tabular}{|l|c|r|}
\hline
\textbf{Coluna 1} & \textbf{Coluna 2} & \textbf{Coluna 3} \\ \hline
Dado 1 & Dado 2 & Dado 3 \\ \hline
\end{tabular}
\fonte{Elaborado pelo autor.}
\end{table}
```

### Adicionar Referências Bibliográficas

Edite `references.bib`:

```bibtex
@book{autor2025titulo,
  title={Título do Livro},
  author={Sobrenome, Nome},
  year={2025},
  publisher={Editora}
}
```

Cite no texto:
```latex
Segundo \cite{autor2025titulo}, ...
```

## 🐛 Troubleshooting

### Erro: "File 'unifacens.sty' not found"

**Solução:** Crie um arquivo `lib/unifacens.sty` vazio ou remova a linha `\usepackage{lib/unifacens}` do preâmbulo.

### Erro: "Undefined control sequence"

**Solução:** Verifique se todos os pacotes estão instalados. No MiKTeX, deixe a opção "Install packages on-the-fly" habilitada.

### Erro na Bibliografia

**Solução:** Certifique-se de executar:
1. PDFLaTeX
2. BibTeX
3. PDFLaTeX (2x)

### Sumário não atualiza

**Solução:** Compile o documento 2-3 vezes seguidas. O LaTeX precisa de múltiplas compilações para resolver todas as referências cruzadas.

### Acentuação incorreta

**Solução:** Certifique-se de que o arquivo está salvo em UTF-8. No TeXstudio: `Edit → Preferences → Editor → Default Character Encoding → UTF-8`.

## 📚 Recursos Adicionais

- **abnTeX2 Manual:** http://www.abntex.net.br/
- **LaTeX Wikibook:** https://en.wikibooks.org/wiki/LaTeX
- **Overleaf Learn:** https://www.overleaf.com/learn
- **ABNT NBR 14724:2011:** Norma para trabalhos acadêmicos

## 📧 Suporte

Para dúvidas sobre:
- **Conteúdo técnico:** Consulte os arquivos `.md` na pasta raiz
- **Formatação LaTeX:** Consulte a documentação do abnTeX2
- **Normas ABNT:** Consulte a biblioteca da instituição

## 📄 Licença

Este documento foi desenvolvido para fins acadêmicos seguindo as normas da ABNT e o modelo da UniFacens.

---

**Última atualização:** 2025
**Versão do abnTeX2:** 1.9.7
**Compilador recomendado:** PDFLaTeX
