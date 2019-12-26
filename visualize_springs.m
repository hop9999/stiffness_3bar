function [] = visualize_springs(x,conect,colors,width,varargin)
Parser = inputParser;
Parser.addOptional('alpha', 0.25);
Parser.parse(varargin{:});
alpha = Parser.Results.alpha;
for k = 1:length(conect)
    con = conect{k};
    s = size(con);
    for i = 1:s(1)
        i1 = con(i,1);
        i2 = con(i,2);
        x
        colors{k}
        p = plot3([x(i1,1),x(i2,1)], [x(i1,2),x(i2,2)],[x(i1,3),x(i2,3)],'Color', colors{k}, 'LineWidth',width{k});
        p.Color(4) = alpha{k};
    end
end