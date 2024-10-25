# RRT do SSL da ROBOIME
> Repositório para o desenvolvimento do RRT em C++

## Índice
- [Sobre o Projeto](#sobre-o-projeto)
- [Requisitos](#requisitos)
- [Como usar no Visual Studio](#como-usar-no-visual-studio)

## Sobre o Projeto
Este projeto é destinado ao desenvolvimento de uma implementação de **RRT (Rapidly-exploring Random Tree)** em C++ para o **Small Size League (SSL)** da **ROBOIME**.

## Requisitos
Este projeto utiliza o gerenciador de pacotes **Vcpkg** para instalar as bibliotecas Flann e lz4.

## Como usar no Visual Studio

### Instale o Vcpkg

1. **Clone o Vcpkg**:
   ```bash
   git clone https://github.com/microsoft/vcpkg.git
1. No windows entre no diretório do Vcpkg e execute:
	```bash
   ./bootstrap-vcpkg.bat

### Instale as biliotecas Flann e lz4
1. No diretório do Vcpkg, instale o flann e o lz4
	```bash
	./vcpkg install flann
	./vcpkg install lz4
1. Com o projeto aberto no Visual Studio, vá em Projeto -> RRT Propriedade, vá em diretórios VC++.<br><br>
	Em diretórios de inclusão coloque:
	```bash
	C:\path\to\vcpkg\installed\x64-windows\include
	```
	Em diretórios de Bivliotecas coloque:
	```bash
	C:\path\to\vcpkg\installed\x64-windows\lib
	```
2. Repita os passos para o Flann e o lz4 na arquitetura x86

---

Lembrando que o Flann e o lb4 podem sem instalados sem o uso do Vcpkg, mas recomendo seu uso
